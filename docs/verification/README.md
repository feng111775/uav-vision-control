# PX4 双摄闭环历史验证

本目录记录较早的 `red_target` 双摄视觉伺服飞行，不是当前 QR 货架逐码验收。
它解决前视接近、下视红区对准和 PX4 Land 的基础闭环验证；当前 QR 结果见
`docs/results/qr_shelf_sequential/`。

历史环境为 Ubuntu 24.04、ROS 2 Jazzy、PX4 v1.17.0、Gazebo Sim 8.11.0，
机型 `gz_x500_downward_camera`，唯一正式控制节点
`vision_offboard_controller.py`。历史状态为：

```text
WAITING → PRESTREAM → TAKEOFF → VISION_CONTROL
→ SEARCH/front → DOWN_ACQUIRE/front → ALIGN/down → LAND → DISARM
```

精选 PNG 展示轨迹、高度和视觉误差；完整 rosbag/ULog 未提交。复查仓库：

```bash
git clone -b feature/qr-shelf-sitl \
  https://github.com/feng111775/uav-vision-control.git \
  ~/px4_ros2_ws
cd ~/px4_ros2_ws
file docs/verification/*.png
cat docs/verification/final_metrics.txt
```

构建和回归：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/px4_ros2_ws/install/setup.bash
colcon test
colcon test-result --verbose
```

当前 QR GUI 的三个终端：

```bash
# 终端 1
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
TARGET_QR_ID=10 ./simulation/scripts/run_qr_sitl_gui.sh
```

```bash
# 终端 2
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /vision/qr/debug_image
```

```bash
# 终端 3
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo /vision/qr/inventory
```

无图像用 `ros2 topic hz /camera/front/image_raw` 排查；无 PX4 状态用
`ss -lunp | grep ':8888'` 排查。停止命令是
`~/px4_ros2_ws/simulation/scripts/stop_qr_sitl.sh`。历史结果不代表真实
Pixhawk、相机、激光、树莓派或装桨飞行已经验证。
