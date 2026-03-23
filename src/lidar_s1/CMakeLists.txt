cmake_minimum_required(VERSION 3.8)
project(lidars1)

# OpenCV配置
find_package(OpenCV REQUIRED COMPONENTS
  core
  highgui
  imgproc
  imgcodecs
  calib3d
)

# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  image_transport
  sensor_msgs
  cv_bridge
)
ament_export_include_directories(include)

# 构建可执行文件
add_executable(lidars1
  src/tcp_image_client.cpp
)

# 链接库
target_link_libraries(lidars1
  ${OpenCV_LIBS}
  image_transport::image_transport
  cv_bridge::cv_bridge
  rclcpp::rclcpp
)

# 构建去畸变节点
add_executable(image_undistort
  src/image_undistort_node.cpp
)

target_link_libraries(image_undistort
  ${OpenCV_LIBS}
  image_transport::image_transport
  cv_bridge::cv_bridge
  rclcpp::rclcpp
)

# 安装配置
install(TARGETS lidars1 image_undistort
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装相机标定配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
