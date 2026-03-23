# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS driver package for the LiDAR S1 sensor - a hardware device combining a Livox Mid-360 LiDAR with dual fisheye cameras (1920x1200@10Hz each). The driver receives compressed images via TCP from the device and publishes them as ROS topics.

## Build System

The package supports both ROS1 and ROS2 from a single codebase using conditional compilation. The build system automatically detects which ROS version to use via the `$ROS_VERSION` environment variable.

**ROS1 (Noetic):**
```bash
cd ~/catkin_ws_lidar/
catkin_make
source devel/setup.bash
```

**ROS2 (Foxy/Humble/Iron):**
```bash
cd ~/catkin_ws_lidar/
colcon build
source install/setup.bash
```

## Running the Driver

**TCP driver only (no undistortion):**
```bash
ros2 launch lidars1 lidars1.launch.py
# With custom parameters:
ros2 launch lidars1 lidars1.launch.py server_ip:=192.168.1.2 server_port:=8888 topic_prefix:=s1_01
```

**TCP driver + undistortion + static TF (no RViz):**
```bash
ros2 launch lidars1 undistort_lidars1.launch.py
# With custom parameters:
ros2 launch lidars1 undistort_lidars1.launch.py server_ip:=192.168.1.2 server_port:=8888 topic_prefix:=s1_01
```

## Testing Without ROS

For debugging TCP connectivity without ROS:
```bash
python3 script/tcp_ros2_display.py
```

## Architecture

### Data Flow
```
Hardware Device (192.168.1.2:8888)
    ↓ TCP/IP over RNDIS (USB 3.0)
TCPImageClient node (src/tcp_image_client.cpp)
    ↓ Deserialize custom ROS message format
Split vertically stacked image (1920x2400)
    ↓ Publish separate streams
/<topic_prefix>/left/image_raw  (1920x1200, default: /s1_01/left/image_raw)
/<topic_prefix>/right/image_raw (1920x1200, default: /s1_01/right/image_raw)
    ↓ ImageUndistortNode (src/image_undistort_node.cpp)
/<topic_prefix>/left/image_rect        (undistorted)
/<topic_prefix>/right/image_rect       (undistorted)
/<topic_prefix>/left/camera_info_rect  (rectified camera info)
/<topic_prefix>/right/camera_info_rect (rectified camera info)
```

### Key Components

**src/tcp_image_client.cpp** - Main driver node
- `TCPImageClient` class handles all functionality
- `connectWithTimeout()` - Non-blocking TCP connection with 3s timeout
- `reconnectLoop()` - Auto-reconnection thread (runs continuously)
- `receiveImages()` - Main loop: reads 4-byte size header, then serialized CompressedImage
- `deserializeROSMessage()` - Custom parser for device's message format
- Thread-safe with `std::mutex` for socket access and `std::atomic` for state flags

### Launch Files

- `lidars1.launch.py` - TCP driver only, publishes raw images
- `undistort_lidars1.launch.py` - TCP driver + undistortion node + static TF (livox_frame → camera_optical)

Both accept parameters: `server_ip`, `server_port`, `topic_prefix` (default `s1_01`).

### Network Protocol

Device sends images over TCP in this format:
1. 4 bytes: `uint32_t` data size (network byte order)
2. N bytes: Serialized CompressedImage message containing:
   - seq (4 bytes)
   - timestamp sec/nsec (8 bytes)
   - frame_id (length-prefixed string)
   - format (length-prefixed string, typically "jpeg")
   - JPEG data (length-prefixed bytes)

The device sends a vertically stacked dual fisheye image (1920x2400) which the driver splits into left (top half) and right (bottom half) streams.

## Hardware Setup

- Device IP: 192.168.1.2 (fixed in firmware)
- TCP port: 8888 (image stream)
- WebSocket port: 9090 (rosbridge format)
- Connection: USB 3.0 RNDIS (appears as network interface, no driver needed on Linux)
- Host must be on 192.168.1.x subnet

Check device connectivity:
```bash
nmap -sP 192.168.1.1/24
ping 192.168.1.2
```

## Dependencies

- OpenCV 4.x (core, highgui, imgproc, imgcodecs, calib3d)
- ROS2: rclcpp, sensor_msgs, cv_bridge, image_transport, tf2_ros, ament_index_cpp
- Livox SDK2 (for LiDAR data, separate from this image driver)

## Important Notes

- Hardware provides PPS signal for <100ns time synchronization between LiDAR and cameras
- Auto-reconnection is built-in; driver will continuously retry if connection drops
- All image/camera_info topic names share a configurable `topic_prefix` (default `s1_01`)
- Calibration YAML files are loaded from `share/lidars1/config/` at runtime
