# QR 货架逐码 GUI SITL 验收证据

本目录保存最终真实 Gazebo GUI 验收的小型、可审查证据，不包含 rosbag、ULog、
数据集或缓存。`acceptance.json` 记录蛇形顺序、确认时间、扫描位置容差、后续
状态与 PX4 终态；`front_camera_real.png` 是经 `ros_gz_bridge` 从
`/camera/front/image_raw` 获取的真实帧。

`camera_required_negative.json` 记录负向 GUI 验收：临时把整个货架移出真实
前视相机视野但保留原扫描航点。飞机实际进入 HOLD/CONFIRM，确认数仍为 0，
两次重试后 FAILSAFE；随后已恢复原世界。它证明位置和 `scan_index` 不能代替
相机解码。

轨迹由扫描计划给出，但每项确认只来自当前图像中
`QRCodeDetector.detectAndDecodeMulti()` 的文本结果。没有读取 QR PNG、位置
推断、测试 `transition()`、假相机、假二维码或假落地消息。

## 复查环境与证据

```bash
git clone -b feature/qr-shelf-sitl \
  https://github.com/feng111775/uav-vision-control.git \
  ~/px4_ros2_ws
cd ~/px4_ros2_ws
python3 -m json.tool \
  docs/results/qr_shelf_sequential/acceptance.json
python3 -c "import cv2; print(cv2.imread('docs/results/qr_shelf_sequential/front_camera_real.png').shape)"
```

完整复现实验需 Ubuntu 24.04、ROS 2 Jazzy、PX4 v1.17.0、Gazebo GUI、
MicroXRCEAgent 和根 README 的安装步骤。

终端 1：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
TARGET_QR_ID=10 ./simulation/scripts/run_qr_sitl_gui.sh
```

终端 2：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /vision/qr/debug_image
```

终端 3：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo /vision/qr/inventory
```

正常时间线：

```text
QR_SCAN_MOVE → QR_SCAN_HOLD → QR_SCAN_CONFIRM → QR_SCAN_NEXT
... 24 次独立确认 ...
QR_INVENTORY_COMPLETE → TARGET_ACQUIRE → TARGET_APPROACH
→ LASER_ALIGN → LASER_CONFIRM → TRANSIT_TO_LANDING
→ DOWN_ACQUIRE(front) → ALIGN(down) → LAND → DISARM
```

若图片打不开、JSON 不是 24 项或顺序错误，证据校验失败。运行栈用
`~/px4_ros2_ws/simulation/scripts/stop_qr_sitl.sh` 停止。此证据只证明 PC
SITL，不证明真实 Pixhawk、树莓派、Coral、激光或装桨飞行。
