# S1 Sensor Bridge Package

Bridge package for the S1 sensor system, which combines dual fisheye cameras with a Livox MID-360 LiDAR. This package provides unified configuration and launch management for both sensor drivers.

## Overview

This package integrates:
- **lidars1**: Dual fisheye camera driver (TCP image receiver + undistortion)
- **livox_ros_driver2**: Livox MID-360 LiDAR driver

All sensor configurations are managed from a single location in this bridge package.

## Quick Start

### 1. Build the Package

```bash
cd ~/ROS/catkin_ws_lidar
colcon build --packages-select s1_sensor_bridge
source install/setup.bash
```

### 2. Launch All Sensors

```bash
ros2 launch s1_sensor_bridge s1_sensor.launch.py
```

This single command starts:
- Camera TCP client (receives images from 192.168.2.2:8888)
- Image undistortion nodes (for both cameras)
- LiDAR driver (connects to 192.168.2.159)
- Static TF publishers (camera-to-lidar transforms)

## Configuration

All sensor parameters are configured in `config/sensor_config.yaml`:

```yaml
# Network Configuration
camera:
  server_ip: "192.168.2.2"
  server_port: 8888

lidar:
  device_ip: "192.168.2.159"
  host_ip: "192.168.2.1"

# Topic and Frame Configuration
topic_prefix: "s1_01"
lidar_frame: "s1_01_lidar_frame"
left_camera_frame: "s1_01_left_camera_optical"
right_camera_frame: "s1_01_right_camera_optical"

# LiDAR Publishing Parameters
lidar_publish_freq: 10.0
lidar_xfer_format: 0  # 0=PointXYZRTL
```

### Calibration Files

Camera calibration files are maintained in `config/`:
- `left_fisheye.yaml` / `right_fisheye.yaml` - Camera intrinsics
- `left_camera_extrinsic.yaml` / `right_camera_extrinsic.yaml` - Camera-to-LiDAR extrinsics
- `MID360_config.json` - LiDAR network and publishing configuration

## Published Topics

### Camera Topics
- `/<topic_prefix>/left/image_raw` - Left camera raw image
- `/<topic_prefix>/left/image_rect` - Left camera undistorted image
- `/<topic_prefix>/left/camera_info_rect` - Left camera info
- `/<topic_prefix>/right/image_raw` - Right camera raw image
- `/<topic_prefix>/right/image_rect` - Right camera undistorted image
- `/<topic_prefix>/right/camera_info_rect` - Right camera info

### LiDAR Topics
- `/<topic_prefix>/lidar` - Point cloud (sensor_msgs/PointCloud2)
- `/<topic_prefix>/imu` - IMU data (sensor_msgs/Imu)

Default `topic_prefix` is `s1_01`.

## TF Tree

```
s1_01_lidar_frame (root)
├── s1_01_left_camera_optical
└── s1_01_right_camera_optical
```

The LiDAR frame serves as the reference frame for the entire sensor system.

## Verification

### Check Topics
```bash
ros2 topic list
ros2 topic hz s1_01/left/image_raw
ros2 topic hz s1_01/lidar
ros2 topic hz s1_01/imu
```

### Check TF Tree
```bash
ros2 run tf2_ros tf2_echo s1_01_lidar_frame s1_01_left_camera_optical
ros2 run tf2_tools view_frames
```

### Visualize in RViz2
```bash
rviz2
```
- Set Fixed Frame to `s1_01_lidar_frame`
- Add PointCloud2 display (topic: `s1_01/lidar`)
- Add Image display (topic: `s1_01/left/image_rect`)
- Add TF display to see coordinate frames

## Network Setup

Ensure your host machine is configured correctly:

```bash
# Check device connectivity
ping 192.168.2.2   # Camera device
ping 192.168.2.159 # LiDAR device

# Verify network interface
ip addr show
```

The S1 device uses USB 3.0 RNDIS (appears as a network interface). Your host should be on the 192.168.2.x subnet.

## Dependencies

- lidars1 (camera driver package)
- livox_ros_driver2 (LiDAR driver package)
- ROS2 packages: rclpy, launch_ros, ament_index_python

## License

GPL-3.0
