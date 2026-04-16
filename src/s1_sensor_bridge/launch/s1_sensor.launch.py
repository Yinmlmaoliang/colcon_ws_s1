import math
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def rotation_matrix_to_quaternion(r):
    """Convert a 3x3 rotation matrix (row-major flat list) to quaternion [x, y, z, w]."""
    m = r
    tr = m[0] + m[4] + m[8]

    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        w = 0.25 * s
        x = (m[7] - m[5]) / s
        y = (m[2] - m[6]) / s
        z = (m[3] - m[1]) / s
    elif m[0] > m[4] and m[0] > m[8]:
        s = 2.0 * math.sqrt(1.0 + m[0] - m[4] - m[8])
        w = (m[7] - m[5]) / s
        x = 0.25 * s
        y = (m[1] + m[3]) / s
        z = (m[2] + m[6]) / s
    elif m[4] > m[8]:
        s = 2.0 * math.sqrt(1.0 + m[4] - m[0] - m[8])
        w = (m[2] - m[6]) / s
        x = (m[1] + m[3]) / s
        y = 0.25 * s
        z = (m[5] + m[7]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m[8] - m[0] - m[4])
        w = (m[3] - m[1]) / s
        x = (m[2] + m[6]) / s
        y = (m[5] + m[7]) / s
        z = 0.25 * s

    return [x, y, z, w]


def invert_transform(r_flat, t):
    """Invert T_cam_lidar to get T_lidar_cam (parent=lidar_frame, child=camera)."""
    r_inv = [
        r_flat[0], r_flat[3], r_flat[6],
        r_flat[1], r_flat[4], r_flat[7],
        r_flat[2], r_flat[5], r_flat[8],
    ]
    t_inv = [
        -(r_inv[0]*t[0] + r_inv[1]*t[1] + r_inv[2]*t[2]),
        -(r_inv[3]*t[0] + r_inv[4]*t[1] + r_inv[5]*t[2]),
        -(r_inv[6]*t[0] + r_inv[7]*t[1] + r_inv[8]*t[2]),
    ]
    return r_inv, t_inv


def load_extrinsic(yaml_path):
    """Load extrinsic YAML and return inverted transform as static TF args."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    r_cl = data['rotation_matrix']['data']
    t_cl = data['translation']['data']

    r_lc, t_lc = invert_transform(r_cl, t_cl)
    qx, qy, qz, qw = rotation_matrix_to_quaternion(r_lc)

    return t_lc, [qx, qy, qz, qw]


def generate_launch_description():
    # Get bridge package config directory
    bridge_pkg_share = get_package_share_directory('s1_sensor_bridge')
    config_dir = os.path.join(bridge_pkg_share, 'config')

    # Load unified sensor configuration
    sensor_config_path = os.path.join(config_dir, 'sensor_config.yaml')
    with open(sensor_config_path, 'r') as f:
        config = yaml.safe_load(f)

    topic_prefix = config['topic_prefix']
    lidar_frame = config['lidar_frame']

    # --- Camera TCP client node ---
    tcp_client_node = Node(
        package='lidars1',
        executable='lidars1',
        name='lidars1',
        output='screen',
        parameters=[{
            'server_ip': config['camera']['server_ip'],
            'server_port': config['camera']['server_port'],
            'topic_prefix': topic_prefix,
            'use_system_time': config['camera']['use_system_time'],
        }]
    )

    # --- Image undistort node (configs from bridge package) ---
    undistort_node = Node(
        package='lidars1',
        executable='image_undistort',
        name='image_undistort',
        output='screen',
        parameters=[{
            'topic_prefix': topic_prefix,
            'config_package': 's1_sensor_bridge',
        }]
    )

    # --- Static TF: lidar_frame -> left_camera_frame ---
    left_t, left_q = load_extrinsic(
        os.path.join(config_dir, 'left_camera_extrinsic.yaml'))
    left_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_camera_tf',
        arguments=[
            '--x',              str(left_t[0]),
            '--y',              str(left_t[1]),
            '--z',              str(left_t[2]),
            '--qx',             str(left_q[0]),
            '--qy',             str(left_q[1]),
            '--qz',             str(left_q[2]),
            '--qw',             str(left_q[3]),
            '--frame-id',       lidar_frame,
            '--child-frame-id', config['left_camera_frame'],
        ]
    )

    # --- Static TF: lidar_frame -> right_camera_frame ---
    right_t, right_q = load_extrinsic(
        os.path.join(config_dir, 'right_camera_extrinsic.yaml'))
    right_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_camera_tf',
        arguments=[
            '--x',              str(right_t[0]),
            '--y',              str(right_t[1]),
            '--z',              str(right_t[2]),
            '--qx',             str(right_q[0]),
            '--qy',             str(right_q[1]),
            '--qz',             str(right_q[2]),
            '--qw',             str(right_q[3]),
            '--frame-id',       lidar_frame,
            '--child-frame-id', config['right_camera_frame'],
        ]
    )

    # --- LiDAR driver node ---
    livox_config_path = os.path.join(config_dir, 'MID360_config.json')
    lidar_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'user_config_path': livox_config_path,
            'xfer_format': int(config['lidar_xfer_format']),
            'publish_freq': float(config['lidar_publish_freq']),
            'frame_id': lidar_frame,
        }],
        remappings=[
            ('livox/lidar', topic_prefix + '/lidar'),
            ('livox/imu', topic_prefix + '/imu'),
        ]
    )

    # --- Static TF: sensor_link -> lidar_frame ---
    is_upside_down = config.get('is_upside_down', False)
    sensor_link = config.get('sensor_link', topic_prefix + '_sensor_link')

    if is_upside_down:
        qx, qy, qz, qw = 1.0, 0.0, 0.0, 0.0
    else:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

    sensor_link_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor_link_tf',
        arguments=[
            '--x',    '0.0',
            '--y',    '0.0',
            '--z',    '0.0',
            '--qx',   str(qx),
            '--qy',   str(qy),
            '--qz',   str(qz),
            '--qw',   str(qw),
            '--frame-id',       sensor_link,
            '--child-frame-id', lidar_frame,
        ]
    )

    return LaunchDescription([
        tcp_client_node,
        undistort_node,
        left_tf_node,
        right_tf_node,
        lidar_node,
        sensor_link_tf_node,
    ])
