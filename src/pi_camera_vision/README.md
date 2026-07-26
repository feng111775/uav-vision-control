# pi_camera_vision 红色目标输入模块

该 ROS 2 包从桌面视频、USB 摄像头或 Raspberry Pi Picamera2 检测红色目标，
并把七字段检测送入滤波/视觉伺服。它不负责 QR 解码，也不会启动 PX4 控制器；
运行它时必须停止 `h7_bridge_node`，避免两个节点同时发布
`/vision/h7/detection`。

## 环境、获取与构建

桌面支持 Ubuntu 24.04、ROS 2 Jazzy；Picamera2 需要 Raspberry Pi OS 的系统包。

```bash
sudo apt update
sudo apt install -y git python3-opencv python3-numpy \
  python3-colcon-common-extensions python3-rosdep
git clone -b feature/qr-shelf-sitl \
  https://github.com/feng111775/uav-vision-control.git \
  ~/px4_ros2_ws
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/px4_ros2_ws/install/setup.bash
python3 -m pytest -q src/pi_camera_vision/test
```

包结构：

```text
src/pi_camera_vision/
├── config/       # 来源、旋转、翻转和 HSV 参数
├── launch/       # standalone 与完整 pipeline
├── pi_camera_vision/
├── test/
└── tools/
```

## 接口与参数

输出 `/vision/h7/detection`，类型 `std_msgs/msg/Float32MultiArray`：

```text
[valid, cx, cy, width, height, area, confidence]
```

主要参数：`source_type=video|usb|picamera2`、`source`、`device`、
`rotation=0|90|180|270`、水平/垂直翻转、两段红色 HSV 范围以及饱和度/亮度
下限。`pi_camera_pipeline.launch.py` 启动一个检测器、滤波器和视觉伺服，
但不启动 `vision_offboard_controller`。

## 三个终端

先生成仓库内可复现测试视频：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
python3 src/pi_camera_vision/tools/create_test_media.py \
  test_results/pi_camera_test
```

终端 1：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 launch pi_camera_vision pi_camera_pipeline.launch.py \
  source_type:=video \
  source:=test_results/pi_camera_test/red_target.avi
```

终端 2：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo /vision/h7/detection
```

终端 3：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 node list
```

USB 可把终端 1 改为 `source_type:=usb device:=0`；Pi 使用
`source_type:=picamera2`。无检测时检查 `ls -l /dev/video*`、视频路径和 HSV
阈值。各终端 `Ctrl-C` 停止；确认没有残留发布者：

```bash
ros2 topic info -v /vision/h7/detection
```

桌面视频回归已实现；Picamera2、真实镜头标定、现场光照、Coral、Pixhawk 和
装桨飞行尚未完成验证。SITL 自动解锁参数与本包无关，绝不能用于真机。
