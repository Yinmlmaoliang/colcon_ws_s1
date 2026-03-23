import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 's1_01_lidar_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'display_point_cloud_ROS2.rviz')
user_config_path = os.path.join(cur_config_path, 'MID360_config.json')

# extract ros parameters from config file
import json
lidar_topic = 'livox/lidar'
imu_topic = 'livox/imu'
try:
    with open(user_config_path, 'r') as f:
        config_data = json.load(f)
        if 'ros_parameters' in config_data:
            ros_params = config_data['ros_parameters']
            frame_id = ros_params.get('frame_id', frame_id)
            lidar_topic = ros_params.get('lidar_topic', lidar_topic)
            imu_topic = ros_params.get('imu_topic', imu_topic)
except Exception as e:
    print(f"Failed to read ros_parameters from {user_config_path}: {e}")
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params,
        remappings=[
            ('livox/lidar', lidar_topic),
            ('livox/imu', imu_topic)
        ]
        )

    # livox_rviz = Node(
    #         package='rviz2',
    #         executable='rviz2',
    #         output='screen',
    #         arguments=['--display-config', rviz_config_path]
    #     )

    return LaunchDescription([
        livox_driver,
        # livox_rviz,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
