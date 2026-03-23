import math
import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def load_sensor_config(config_dir):
    """Load sensor.yaml and return its values as a dict."""
    path = os.path.join(config_dir, 'sensor.yaml')
    with open(path, 'r') as f:
        return yaml.safe_load(f)


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
    pkg_share = get_package_share_directory('lidars1')
    config_dir = os.path.join(pkg_share, 'config')

    # Read defaults from sensor.yaml so editing the file is enough for most users.
    sensor_cfg = load_sensor_config(config_dir)
    default_server_ip       = sensor_cfg.get('server_ip',           '192.168.2.2')
    default_server_port     = str(sensor_cfg.get('server_port',    8888))
    default_topic_prefix    = sensor_cfg.get('topic_prefix',        's1_01')
    default_lidar_frame     = sensor_cfg.get('lidar_frame',         'livox_frame')
    default_left_cam_frame  = sensor_cfg.get('left_camera_frame',   'left_camera_optical')
    default_right_cam_frame = sensor_cfg.get('right_camera_frame',  'right_camera_optical')

    server_ip_arg = DeclareLaunchArgument(
        'server_ip',
        default_value=default_server_ip,
        description='TCP server IP address (default from sensor.yaml)'
    )
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value=default_server_port,
        description='TCP server port (default from sensor.yaml)'
    )
    topic_prefix_arg = DeclareLaunchArgument(
        'topic_prefix',
        default_value=default_topic_prefix,
        description='Unified prefix for all image/camera_info topics (default from sensor.yaml)'
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value=default_lidar_frame,
        description='TF frame ID of the LiDAR (parent frame for camera TFs, default from sensor.yaml)'
    )
    left_cam_frame_arg = DeclareLaunchArgument(
        'left_camera_frame',
        default_value=default_left_cam_frame,
        description='TF child frame ID for the left camera (default from sensor.yaml)'
    )
    right_cam_frame_arg = DeclareLaunchArgument(
        'right_camera_frame',
        default_value=default_right_cam_frame,
        description='TF child frame ID for the right camera (default from sensor.yaml)'
    )

    tcp_client_node = Node(
        package='lidars1',
        executable='lidars1',
        name='lidars1',
        output='screen',
        parameters=[{
            'server_ip':    LaunchConfiguration('server_ip'),
            'server_port':  LaunchConfiguration('server_port'),
            'topic_prefix': LaunchConfiguration('topic_prefix'),
        }]
    )

    undistort_node = Node(
        package='lidars1',
        executable='image_undistort',
        name='image_undistort',
        output='screen',
        parameters=[{
            'topic_prefix': LaunchConfiguration('topic_prefix'),
        }]
    )

    # Static TF: lidar_frame -> left_camera_frame
    # Extrinsic calibration is in optical convention (Z forward, X right, Y down).
    left_t, left_q = load_extrinsic(
        os.path.join(config_dir, 'left_camera_extrinsic.yaml'))
    left_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='left_camera_tf',
        arguments=[
            '--x',            str(left_t[0]),
            '--y',            str(left_t[1]),
            '--z',            str(left_t[2]),
            '--qx',           str(left_q[0]),
            '--qy',           str(left_q[1]),
            '--qz',           str(left_q[2]),
            '--qw',           str(left_q[3]),
            '--frame-id',     LaunchConfiguration('lidar_frame'),
            '--child-frame-id', LaunchConfiguration('left_camera_frame'),
        ]
    )

    # Static TF: lidar_frame -> right_camera_frame
    right_t, right_q = load_extrinsic(
        os.path.join(config_dir, 'right_camera_extrinsic.yaml'))
    right_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='right_camera_tf',
        arguments=[
            '--x',            str(right_t[0]),
            '--y',            str(right_t[1]),
            '--z',            str(right_t[2]),
            '--qx',           str(right_q[0]),
            '--qy',           str(right_q[1]),
            '--qz',           str(right_q[2]),
            '--qw',           str(right_q[3]),
            '--frame-id',     LaunchConfiguration('lidar_frame'),
            '--child-frame-id', LaunchConfiguration('right_camera_frame'),
        ]
    )

    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        topic_prefix_arg,
        lidar_frame_arg,
        left_cam_frame_arg,
        right_cam_frame_arg,
        tcp_client_node,
        undistort_node,
        left_tf_node,
        right_tf_node,
    ])
