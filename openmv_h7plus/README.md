# OpenMV Cam H7 Plus 红色色块检测

本目录是 OpenMV 固件端的红色目标检测与 USB CDC 文本协议，实现向 ROS 桥接
输出目标中心、尺寸、面积和工程置信度。它不解码 QR，不控制 PX4，也未参与
当前 GUI SITL 的二维码确认。

## 要求与获取

- OpenMV Cam H7 Plus、OV5640；
- OpenMV 固件 5.0 / MicroPython 1.28；
- Ubuntu 24.04 + ROS 2 Jazzy 用于主机桥接。

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep
git clone -b feature/qr-shelf-sitl \
  https://github.com/feng111775/uav-vision-control.git \
  ~/px4_ros2_ws
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/px4_ros2_ws/install/setup.bash
python3 -m pytest -q src/uav_vision/test/test_h7_parser.py
```

目录：

```text
openmv_h7plus/
├── camera_config.py
├── detector.py
├── main.py
├── protocol.py
└── thresholds.py
```

## 烧录与参数

在 OpenMV IDE 中把五个文件复制到板载根目录，并把 `main.py` 保存为开机脚本。
默认 RGB565/QVGA 320×240、约 20 Hz。`RED_THRESHOLDS` 是 LAB 六元组；
`PIXELS_THRESHOLD` 和 `AREA_THRESHOLD` 过滤噪声。现场必须用 IDE Threshold
Editor 重新标定。

输出格式：

```text
TARGET,valid,cx,cy,width,height,area,confidence
TARGET,1,160,120,50,48,2400,90
TARGET,0,0,0,0,0,0,0
```

ROS 输出 `/vision/h7/detection`，类型
`std_msgs/msg/Float32MultiArray`。`confidence` 是 0～100 工程评分，不是神经
网络概率。

## 三个终端/界面

终端或界面 1：OpenMV IDE 上传并运行，随后断开 IDE 串口。

终端 2：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run uav_vision h7_bridge_node --ros-args \
  -p port:=/dev/ttyACM0 -p baudrate:=115200
```

终端 3：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo /vision/h7/detection
```

若失败，执行 `ls -l /dev/ttyACM*`、`groups`、`ros2 topic info -v
/vision/h7/detection`；确保 OpenMV IDE 和 ROS 不同时占用串口。用 `Ctrl-C`
停止桥接并在 IDE 中停止脚本。

桌面只能测试协议解析，不能导入 `csi`/`pyb`。真实相机方向、阈值、曝光、
断连恢复、持续 FPS、Pixhawk 和装桨飞行尚未验证。任何 SITL 自动解锁配置都
不得移植到真机。
