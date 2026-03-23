# 图像流传输协议对比：自定义 TCP 推流 vs RTSP

> 本文档以 LiDAR S1 设备的实际实现为基础，对比自定义 TCP 二进制推流协议与 RTSP 标准视频流协议，
> 帮助理解两种方案的原理、适用场景及优缺点。

---

## 一、自定义 TCP 推流协议（本项目实际使用）

### 1.1 基本原理

TCP（Transmission Control Protocol，传输控制协议）是面向连接、可靠的字节流传输协议。
LiDAR S1 设备在 TCP 之上定义了一套**私有的二进制帧格式**，将图像数据逐帧推送给客户端。

```
设备端（TCP Server）                    主机端（TCP Client）
      │                                        │
      │  ←── TCP 三次握手 connect ─────────────│
      │                                        │
      │  ──── 帧1：[4B长度][载荷数据] ────────→│
      │  ──── 帧2：[4B长度][载荷数据] ────────→│
      │  ──── 帧3：[4B长度][载荷数据] ────────→│
      │              ...持续推流...             │
```

### 1.2 数据帧格式

每一帧图像数据由两部分构成：

```
┌─────────────────────────────────────────────────────┐
│                    一个完整的帧                       │
├──────────────┬──────────────────────────────────────┤
│  帧头 4字节   │            载荷 N字节                  │
│  Big-Endian  │       自定义二进制序列化消息             │
│  uint32_t    │   （CompressedImage 各字段紧密排列）     │
│  (数据总长度) │                                       │
└──────────────┴──────────────────────────────────────┘
```

**载荷内部字段布局（Little-Endian）：**

```
偏移      类型            字段           说明
+0        uint32_t        seq            帧序列号
+4        int32_t         sec            时间戳（秒）
+8        int32_t         nsec           时间戳（纳秒）
+12       uint32_t        frame_id_len   坐标系ID字符串长度
+16       char[]          frame_id       坐标系ID（无null终止）
+N        uint32_t        format_len     编码格式字符串长度
+N+4      char[]          format         编码格式（如 "jpeg"）
+M        uint32_t        jpeg_len       JPEG数据长度
+M+4      uint8_t[]       jpeg_data      JPEG压缩图像（核心载荷）
```

**注意字节序：**
- 帧头长度字段：**Big-Endian**（网络字节序），需用 `ntohl()` 或 `struct.unpack('!I')` 转换
- 载荷内部字段：**Little-Endian**（主机字节序），直接 `memcpy` 或 `struct.unpack('<I')` 读取

### 1.3 接收流程

```
① recv 4字节 → ntohl() → 得到 data_size
② 分配 data_size 字节缓冲区
③ 循环 recv 直到读满（处理 TCP 分包）
④ 按偏移解析各字段（反序列化）
⑤ JPEG字节 → cv::imdecode() → BGR cv::Mat
⑥ 按 height/2 分割为左/右鱼眼图像
⑦ 发布 ROS Topic
```

### 1.4 关键实现细节

**C++ 核心代码（tcp_image_client.cpp）：**
```cpp
// 读取帧头
uint32_t data_size;
readWithTimeout(&data_size, sizeof(data_size), 1);
data_size = ntohl(data_size);  // 大端 → 小端

// 读取完整载荷（循环recv处理TCP分包）
std::vector<uint8_t> buffer(data_size);
readWithTimeout(buffer.data(), data_size, 3);

// JPEG解码
cv::Mat cv_img = cv::imdecode(compressed_img.data, cv::IMREAD_COLOR);
```

**Python 核心代码（tcp_ros2_display.py）：**
```python
data_size = struct.unpack('!I', header)[0]   # 帧头：大端序
jpeg_len  = struct.unpack_from('<I', ...)[0]  # 载荷内：小端序
cv_img    = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
```

### 1.5 编码类型总结

| 阶段 | 编码/格式 | 说明 |
|------|-----------|------|
| 设备端原始图像 | RAW（设备内部） | 传感器输出的原始 Bayer 或 YUV 数据 |
| 网络传输 | **JPEG** 压缩 | 两路鱼眼垂直拼接后 JPEG 压缩，降低带宽 |
| 解码后 | **BGR8** (cv::Mat) | OpenCV IMREAD_COLOR 默认输出 BGR 三通道 |
| ROS 发布 | **sensor_msgs/Image BGR8** | 原始像素格式，供其他节点订阅处理 |

---

## 二、RTSP 协议（行业标准视频流）

### 2.1 基本原理

RTSP（Real-Time Streaming Protocol，实时流传输协议，RFC 2326）是一个**应用层控制协议**，
专为多媒体流设计。它本身不传输媒体数据，而是作为"遥控器"控制媒体流的传输。

实际视频数据通过 **RTP**（Real-time Transport Protocol）传输，
传输质量反馈通过 **RTCP**（RTP Control Protocol）完成。

```
┌─────────────────────────────────────────────────────────────┐
│                       RTSP 协议栈                            │
├──────────────────────────────────────────────────────────────┤
│  应用层    │  RTSP (TCP 554)  │  控制信令：DESCRIBE/SETUP/PLAY │
│           │  RTP  (UDP)      │  媒体数据传输                  │
│           │  RTCP (UDP)      │  质量反馈、同步                 │
├──────────────────────────────────────────────────────────────┤
│  传输层    │  TCP（控制） + UDP（数据，默认）                   │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 会话建立流程

```
客户端                              RTSP服务端（摄像头/流媒体服务器）
   │                                        │
   │  OPTIONS rtsp://192.168.1.2/live RTSP/1.0   │
   │ ──────────────────────────────────────→ │
   │ ← 200 OK (Public: DESCRIBE, SETUP, PLAY)│
   │                                        │
   │  DESCRIBE rtsp://192.168.1.2/live       │
   │ ──────────────────────────────────────→ │
   │ ← 200 OK + SDP（流描述：编码/分辨率/帧率）│
   │                                        │
   │  SETUP（指定RTP端口）                    │
   │ ──────────────────────────────────────→ │
   │ ← 200 OK + Session ID                  │
   │                                        │
   │  PLAY（开始播放）                        │
   │ ──────────────────────────────────────→ │
   │ ← 200 OK                               │
   │                                        │
   │ ←═══ RTP包：H.264/H.265/MJPEG 数据 ════│  ← 持续推流
   │ ←═══ RTP包 ════════════════════════════│
   │              ...                       │
   │  TEARDOWN（结束会话）                    │
   │ ──────────────────────────────────────→ │
```

### 2.3 RTSP 使用的视频编码

| 编码格式 | 特点 | 典型场景 |
|----------|------|----------|
| **H.264** (AVC) | 最广泛支持，硬件加速普遍 | IP摄像头、NVR录像 |
| **H.265** (HEVC) | 相同质量带宽减半，计算量更大 | 高分辨率4K流 |
| **MJPEG** | 每帧独立JPEG，无帧间依赖 | 低延迟、易编辑场景 |
| **H.266** (VVC) | 最新标准，压缩率最高 | 尚未普及 |

### 2.4 SDP 媒体描述示例

```
v=0
o=- 0 0 IN IP4 192.168.1.2
s=Live Stream
t=0 0
m=video 0 RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1; sprop-parameter-sets=Z0...
a=framerate:30
```

---

## 三、两种协议对比

| 维度 | 自定义 TCP 推流（本项目） | RTSP / RTP |
|------|--------------------------|------------|
| **协议标准** | 私有协议，无标准 | 标准协议（RFC 2326/3550） |
| **传输层** | TCP（可靠） | 控制：TCP；数据：UDP（默认） |
| **连接模型** | 设备主动等待，客户端 connect | 客户端主动 DESCRIBE/SETUP/PLAY |
| **视频编码** | JPEG（帧内压缩） | H.264/H.265/MJPEG（可选） |
| **帧间压缩** | 无（每帧独立JPEG） | H.264/H.265 有帧间压缩（P/B帧） |
| **带宽效率** | 中等（仅帧内压缩） | 高（帧间压缩可降低 60-80% 带宽） |
| **延迟** | 极低（无缓冲、无编解码延迟） | 较低（通常 100ms~500ms） |
| **时间同步** | 自带时间戳字段（PPS硬件同步） | RTCP NTP时间戳同步 |
| **丢包处理** | TCP 保证可靠，自动重传 | UDP默认不重传，NACK可选 |
| **兼容性** | 仅自己的客户端可用 | VLC/ffplay/GStreamer等通用播放器 |
| **硬件支持** | 不需要专用编解码硬件 | H.264/H.265 通常需编码芯片 |
| **实现复杂度** | 简单（纯 TCP socket） | 较复杂（信令+RTP打包+会话管理） |
| **适用场景** | 嵌入式设备、专用ROS驱动 | 通用监控、媒体服务器、多客户端 |

---

## 四、本项目选择自定义 TCP 的原因分析

### 4.1 为什么不用 RTSP

1. **强时间同步需求**：LiDAR S1 通过 PPS 信号实现 LiDAR 与相机的亚微秒级时间同步，
   自定义格式可以直接在载荷中携带硬件时间戳，RTSP/RTP 的 NTP 时间戳精度不足。

2. **ROS 消息格式直通**：设备端直接序列化 `CompressedImage` ROS 消息格式，
   客户端零转换开销反序列化，与 ROS 生态深度集成。

3. **嵌入式设备资源受限**：H.264 编码需要专用 ISP/编码芯片，JPEG 压缩对嵌入式 CPU 更友好。

4. **单一客户端场景**：本驱动为一对一连接，不需要 RTSP 的多客户端会话管理能力。

5. **实现简单可控**：整个协议仅 100 行 C++ / Python 代码实现，调试和维护成本低。

### 4.2 如果要升级为 RTSP 的改造思路

若未来需要让 VLC、Web 浏览器等通用客户端访问图像流，可考虑：

```bash
# 方案一：GStreamer 将 ROS Topic 转为 RTSP 流
gst-launch-1.0 rtspsrc location=rtsp://192.168.1.2/live ! ...

# 方案二：FFmpeg 将 TCP 流转封装为 RTSP
ffmpeg -i tcp://192.168.1.2:8888 -c:v copy -f rtsp rtsp://localhost/stream

# 方案三：在 ROS2 中用 image_transport 的 compressed 插件
# 订阅 /fisheye/left/image_raw → web_video_server 提供 HTTP/MJPEG 流
```

---

## 五、快速验证命令

```bash
# 验证设备TCP端口是否开放
nmap -sT -p 8888 192.168.1.2

# 用 nc 测试TCP连通性
nc -zv 192.168.1.2 8888

# 如果是RTSP，用 VLC 打开
vlc rtsp://192.168.1.2:554/live

# 如果是RTSP，用 ffprobe 查看流信息
ffprobe rtsp://192.168.1.2:554/live

# 抓包观察协议类型
sudo tcpdump -i usb0 host 192.168.1.2 and port 8888 -n

# 直接运行Python脚本验证本项目TCP协议
python3 script/tcp_ros2_display.py
```

---

## 六、总结

| | 自定义 TCP（本项目） | RTSP |
|---|---|---|
| **本质** | 私有二进制帧协议，承载 JPEG | 标准多媒体流控制协议，承载 H.264 等 |
| **优势** | 低延迟、与ROS深度集成、实现简单 | 标准化、多客户端、带宽效率高 |
| **劣势** | 不通用、无帧间压缩 | 复杂度高、延迟略高、需编解码硬件 |
| **推荐场景** | 机器人/ROS专用驱动、硬件时间同步 | 通用监控、流媒体服务、多平台接入 |
