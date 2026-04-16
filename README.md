> ⚠️**本项目目前处于测试阶段，在使用过程中随时反馈任何问题，随时解决！**

# colcon_ws_s1 — S1 传感器 ROS 2 工作空间

本工作空间包含 Livox MID-360 激光雷达与双目鱼眼相机的 ROS 2 驱动与桥接功能包，支持多设备并行运行，并提供统一的时间戳、坐标系与话题管理。

---

## 功能特性

- **统一命名空间与参考帧**：支持多设备同时启动，每个设备可自定义命名空间，避免话题与坐标系冲突。
- **相机内参发布**：实时发布原始鱼眼相机内参及去畸变后的内参（`camera_info` / `camera_info_rect`）。
- **传感器外参发布**：以静态 TF（Static TF）形式发布左右相机与激光雷达之间的相对位姿关系。
- **一键启动**：作为桥接包统一管理相机与雷达的启动流程。
- **正装 / 倒装自动适配**：在物理硬件帧（`lidar_frame`）与逻辑传感器帧（`sensor_link`）之间插入抽象层，始终保持右手坐标系。

### TF 树结构

```
s1_01_sensor_link              ← 逻辑帧，永远正装
└── s1_01_lidar_frame          ← 物理雷达帧，倒装时绕 X 轴翻转 180°
    ├── s1_01_left_camera_optical   （外参不变）
    └── s1_01_right_camera_optical  （外参不变）
```

倒装时只需在配置文件中设置 `is_upside_down: true`，重新编译后即可生效；`undistort_lidars1.launch.py` 无需任何修改，翻转变换会自动传递至整个子树。

> **注**：点云与坐标轴会随之翻转，但原始视频流（`image_raw`）依然以物理方向（倒置）输出。这是有意为之——若在内存中翻转图像，图像坐标系 `(u, v)` 会随之改变，进而影响内参与算法的一致性。调试可视化层（Rviz / Web UI）可自行进行逻辑翻转显示。

---

## 时间戳策略

| 传感器 | 时间戳来源 | 说明 |
|--------|-----------|------|
| 激光雷达 | PTP 同步后的硬件时间 | 通过 `ptp4l` 将系统时间同步到雷达 |
| 相机 | ROS 节点系统时间 | `use_system_time: true` 时使用 `node_->now()` |

> ⚠️ **准确性与同步性待定**：目前 PTP + `use_system_time` 方案在多传感器时间戳一致性方面尚未经过严格验证，以下说明供参考，最终效果以实际使用反馈为准。

---

## 快速开始

### 1. 克隆仓库

```bash
git clone https://github.com/Yinmlmaoliang/colcon_ws_s1.git
cd colcon_ws_s1
```

---

### 2. 安装 Livox-SDK2

雷达驱动依赖 Livox-SDK2，请参考以下仓库的说明完成安装：

> 📦 [https://gitee.com/nochain/lidar_s1](https://gitee.com/nochain/lidar_s1)

通常步骤如下：

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

---

### 3. 配置网络接口

MID-360 默认工作在 `192.168.2.x` 网段(根据实际情况)，需要将主机网卡配置到同一网段。

**查看可用网卡名称：**

```bash
ip link show
```

**临时配置 IP（以网卡 `enxb2d65bc54aef` 为例）：**

```bash
# 修改为自己的网卡名称
sudo ip addr add 192.168.2.1/24 dev enxb2d65bc54aef
sudo ip link set enxb2d65bc54aef up
```
参考以下仓库的UI界面说明完成安装：

> 📦 [https://gitee.com/nochain/lidar_s1](https://gitee.com/nochain/lidar_s1)

**验证连通性：**

```bash
ping 192.168.2.159   # 雷达默认 IP
```

#### 3.1 使用 PTP 将系统时间同步至雷达

MID-360 支持 PTP 协议。运行以下命令，使雷达以主机系统时间为参考同步其内部时钟：

```bash
# 修改为自己的网卡名称
sudo ptp4l -i enxb2d65bc54aef -l 6 -m -S
```

| 参数 | 说明 |
|------|------|
| `-i enxb2d65bc54aef` | 指定用于 PTP 同步的网络接口（替换为实际网卡名） |
| `-l 6` | 日志级别（6 = debug，可改为 5 减少输出） |
| `-m` | 将日志打印到标准输出 |
| `-S` | 使用软件时间戳（无硬件 PTP 支持时使用） |

> 该命令需在启动雷达驱动前运行，并保持在后台持续运行。

---

### 4. 修改配置文件

所有配置文件位于 `src/s1_sensor_bridge/config/` 目录下。

> **重要提示**：使用默认配置文件可以正常启动并获取基础数据。若算法需要使用高精度内参或精确外参，对于不同设备序号，必须重新标定并替换对应配置文件，重新编译后生效。

#### 4.1 `MID360_config.json` — 雷达网络与运行配置

**必须根据实际网络环境修改**，否则雷达无法连接。

```json
{
  "ros_parameters": {
    "frame_id": "s1_01_lidar_frame",   // 雷达点云的坐标帧 ID
    "lidar_topic": "s1_01/lidar",      // 点云话题名
    "imu_topic": "s1_01/imu"          // IMU 话题名
  },
  "lidar_summary_info": {
    "lidar_type": 8                    // 8 = MID-360
  },
  "MID360": {
    "lidar_net_info": {
      "cmd_data_port": 56100,
      "push_msg_port": 56200,
      "point_data_port": 56300,
      "imu_data_port": 56400,
      "log_data_port": 56500
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.2.1",   // ⭐ 主机 IP，需与网卡配置一致
      "cmd_data_port": 56101,
      "push_msg_ip": "192.168.2.1",
      "push_msg_port": 56201,
      "point_data_ip": "192.168.2.1",
      "point_data_port": 56301,
      "imu_data_ip": "192.168.2.1",
      "imu_data_port": 56401,
      "log_data_ip": "",
      "log_data_port": 56501
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.2.159",           // ⭐ 雷达设备 IP（查看雷达标签或用 Livox Viewer 确认）
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "extrinsic_parameter": {         // 雷达自身坐标系偏移（一般保持为 0）
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "x": 0, "y": 0, "z": 0
      }
    }
  ]
}
```

**需要修改的字段：**

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `host_net_info.*_ip` | `192.168.2.1` | 主机网卡在雷达网段的 IP |
| `lidar_configs[0].ip` | `192.168.2.159` | 雷达设备实际 IP |

---

#### 4.2 `sensor_config.yaml` — 传感器全局配置 ⭐

这是最核心的配置文件，控制网络连接、安装方向、命名空间与发布参数。

```yaml
# ── 网络配置 ──────────────────────────────────────────────────────────────────
camera:
  server_ip: "192.168.2.2"      # ⭐ 相机流媒体服务器 IP（需与实际相机 IP 一致）
  server_port: 8888              # 相机流媒体服务器端口
  use_system_time: true          # ⭐ true = 使用 ROS 节点系统时间替代设备时间戳
                                 #    配合 ptp4l 可使雷达与相机时间戳对齐（效果待验证）

lidar:
  device_ip: "192.168.2.159"    # ⭐ 雷达设备 IP（需与 MID360_config.json 一致）
  host_ip: "192.168.2.1"        # ⭐ 主机 IP（需与网卡配置一致）

# ── 安装方向 ──────────────────────────────────────────────────────────────────
is_upside_down: true             # ⭐ 设备倒装时设为 true；正装设为 false
                                 #    倒装时 sensor_link → lidar_frame 绕 X 轴翻转 180°
                                 #    修改后需重新 colcon build

# ── 话题与坐标帧命名 ──────────────────────────────────────────────────────────
topic_prefix: "s1_01"                          # 话题前缀，多设备时需唯一（如 s1_02）
sensor_link: "s1_01_sensor_link"               # 逻辑传感器帧（始终正装）
lidar_frame: "s1_01_lidar_frame"               # 物理雷达帧
left_camera_frame: "s1_01_left_camera_optical" # 左相机光学帧
right_camera_frame: "s1_01_right_camera_optical" # 右相机光学帧

# ── 雷达发布参数 ──────────────────────────────────────────────────────────────
lidar_publish_freq: 10.0         # 点云发布频率（Hz）
lidar_xfer_format: 0             # 点云格式：0 = PointXYZRTL
```

**参数说明汇总：**

| 参数 | 类型 | 说明 | 是否必须修改 |
|------|------|------|-------------|
| `camera.server_ip` | string | 相机 IP 地址 | ✅ 是 |
| `camera.server_port` | int | 相机端口 | 通常保持默认 |
| `camera.use_system_time` | bool | 使用系统时间戳 | 推荐 `true` |
| `lidar.device_ip` | string | 雷达 IP | ✅ 是 |
| `lidar.host_ip` | string | 主机网卡 IP | ✅ 是 |
| `is_upside_down` | bool | 设备是否倒装 | 根据安装方式 |
| `topic_prefix` | string | 话题前缀 | 多设备时需唯一 |
| `sensor_link` | string | 逻辑传感器帧名 | 通常保持默认 |
| `lidar_frame` | string | 物理雷达帧名 | 通常保持默认 |
| `lidar_publish_freq` | float | 点云发布频率 (Hz) | 按需调整 （待测试） |
| `lidar_xfer_format` | int | 点云数据格式 | 通常保持 `0` |

---

#### 4.3 相机内参文件 — `left_fisheye.yaml` / `right_fisheye.yaml`

存放左右鱼眼相机的内参矩阵与畸变系数，用于 `camera_info` 话题发布和图像去畸变。

> 默认提供示例标定值，正式使用时**建议重新标定**并替换。

#### 4.4 相机外参文件 — `left_camera_extrinsic.yaml` / `right_camera_extrinsic.yaml`

存放左右相机相对于雷达坐标系的平移与旋转参数，由桥接节点读取并发布为静态 TF。

> 外参挂载在 `lidar_frame` 下，倒装翻转通过 `sensor_link → lidar_frame` 自动传递，**外参文件本身无需修改**。

---

### 5. 编译工作空间

在工作空间根目录下执行：

```bash
colcon build
source install/setup.bash
```

> 修改 `is_upside_down` 或其他 `sensor_config.yaml` 中的参数后，均需重新执行 `colcon build`。

---

### 6. 运行

#### 6.1 启动传感器节点

```bash
# 加载环境变量
source install/setup.bash

# 启动 S1 传感器桥接节点
ros2 launch s1_sensor_bridge s1_sensor.launch.py
```

#### 6.2 发布的话题列表

启动成功后，以下话题将被统一发布：

```
# 点云与 IMU
/s1_01/lidar
/s1_01/imu

# 左相机
/s1_01/left/camera_info
/s1_01/left/camera_info_rect
/s1_01/left/image_raw
/s1_01/left/image_raw/compressed
/s1_01/left/image_rect

# 右相机
/s1_01/right/camera_info
/s1_01/right/camera_info_rect
/s1_01/right/image_raw
/s1_01/right/image_raw/compressed
/s1_01/right/image_rect

# 静态 TF（外参）
/tf_static
```

#### 6.3 验证

```bash
# 查看话题列表
ros2 topic list

# 查看点云频率
ros2 topic hz /s1_01/lidar

# 可视化（启动 Rviz2 并添加 PointCloud2 / Image 插件）
rviz2
```

---

## 目录结构

```
colcon_ws_s1/
├── src/
│   ├── s1_sensor_bridge/          # 传感器桥接主功能包
│   │   ├── config/
│   │   │   ├── MID360_config.json          # ⭐ 雷达网络配置（必须修改）
│   │   │   ├── sensor_config.yaml          # ⭐ 传感器全局配置（必须修改）
│   │   │   ├── left_fisheye.yaml           # 左相机内参
│   │   │   ├── right_fisheye.yaml          # 右相机内参
│   │   │   ├── left_camera_extrinsic.yaml  # 左相机外参
│   │   │   └── right_camera_extrinsic.yaml # 右相机外参
│   │   └── launch/
│   │       └── s1_sensor.launch.py
│   ├── lidar_s1/                  # 雷达点云处理功能包
│   └── livox_ros_driver2/         # Livox 官方 ROS 2 驱动
```

---

## 参考

- 网络配置与设备信息获取：[https://gitee.com/nochain/lidar_s1](https://gitee.com/nochain/lidar_s1)
- Livox-SDK2：[https://github.com/Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2)