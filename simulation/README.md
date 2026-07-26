# PX4 QR shelf GUI SITL 与 overlay

本模块提供 PX4 v1.17.0 的 `qr_shelf_world`、4×6 QR 货架、前/下视相机机型、
airframe 和可靠启停脚本。Gazebo 主窗口是第三人称视角；机载相机及识别框必须
另开 `rqt_image_view`。

## 从零安装

```bash
sudo apt update
sudo apt install -y git ros-jazzy-ros-gz-bridge \
  ros-jazzy-rqt-image-view python3-colcon-common-extensions python3-rosdep
git clone -b feature/qr-shelf-sitl \
  https://github.com/feng111775/uav-vision-control.git \
  ~/px4_ros2_ws
git clone --recursive -b v1.17.0 \
  https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl_default
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
./simulation/scripts/install_px4_overlay.sh ~/PX4-Autopilot
./simulation/scripts/install_px4_overlay.sh ~/PX4-Autopilot --apply
```

`px4_overlay/` 保留 PX4 根目录相对路径；dry-run 会列出文件，`--apply` 才安装。
脚本验证 v1.17，并在缺失时加入 airframe `4022_gz_x500_downward_camera`。

## 三个终端

终端 1，一键 GUI 栈：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
TARGET_QR_ID=10 ./simulation/scripts/run_qr_sitl_gui.sh
```

终端 2，真实前视识别：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /vision/qr/debug_image
```

终端 3，状态与 inventory：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo /vision/qr/inventory
```

关键参数：`TARGET_QR_ID=1..24`、`PX4_AUTOPILOT_ROOT`、`QR_SITL_LOG_DIR`。
扫描点只规划运动；确认只来自前视图像连续解码。遮挡当前码应看到 RETRY，
重试耗尽应 FAILSAFE，不能继续 24/24。

排查和停止：

```bash
cd ~/px4_ros2_ws
gz topic -l | grep /camera
ros2 topic hz /camera/front/image_raw
ss -lunp | grep ':8888'
./simulation/scripts/stop_qr_sitl.sh
```

成功标准是 24 个独立确认、选择目标、下视对准、Land、Disarmed 且
`failsafe=false`。自动 Offboard/自动解锁仅限本机 SITL，绝不能用于真实
Pixhawk；真机硬件、标定、接管和飞行尚未验证。
