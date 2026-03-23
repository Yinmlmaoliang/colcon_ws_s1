import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lidars1')
    sensor_cfg_path = os.path.join(pkg_share, 'config', 'sensor.yaml')
    with open(sensor_cfg_path, 'r') as f:
        sensor_cfg = yaml.safe_load(f)
    default_server_ip    = sensor_cfg.get('server_ip',    '192.168.2.2')
    default_server_port  = str(sensor_cfg.get('server_port', 8888))
    default_topic_prefix = sensor_cfg.get('topic_prefix', 's1_01')

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
        description='Unified prefix for all published image and camera_info topics (default from sensor.yaml)'
    )

    tcp_client_node = Node(
        package='lidars1',
        executable='lidars1',
        name='lidars1',
        output='screen',
        parameters=[{
            'server_ip': LaunchConfiguration('server_ip'),
            'server_port': LaunchConfiguration('server_port'),
            'topic_prefix': LaunchConfiguration('topic_prefix'),
        }]
    )

    return LaunchDescription([
        server_ip_arg,
        server_port_arg,
        topic_prefix_arg,
        tcp_client_node,
    ])