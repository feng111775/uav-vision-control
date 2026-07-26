# QR shelf 历史实验与后端基准

本目录保存逐码任务之前的历史训练、相机和 headless SITL 证据，用于解释资产
修复和后端选择；当前最终验收请看
`docs/results/qr_shelf_sequential/`。这里的旧固定视角或 headless 结果不能替代
当前 24 点 GUI 逐码验收。

## 获取与检查

```bash
git clone -b feature/qr-shelf-sitl \
  https://github.com/feng111775/uav-vision-control.git \
  ~/px4_ros2_ws
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/px4_ros2_ws/install/setup.bash
python3 simulation/scripts/benchmark_qr_backend.py --help
python3 -m pytest -q src/uav_vision/test/test_qr_core.py
```

目录中的 JSON 输入是历史运行指标，PNG 是精选结果图；不发布 ROS 话题。
OpenCV 负责最终 ID 解码，HOG+SVM 只提供候选区域。历史 50 帧验证读取
Gazebo `/camera/shelf_validation/image`，没有读取原始二维码 PNG。

已实现并保留：

- 0.19 m QR COLLADA 几何和材质方向修复；
- OpenCV/混合后端延迟与 held-out 数据；
- 早期 headless 失败和后续修复记录；
- 小型可提交证据。

当前正式逐码成功标准是根 README 所述蛇形 24/24、目标 QR 10、下视切换、
Land/Disarm、`failsafe=false`，而不是本目录中的单帧定位率。

重新运行正式栈的三个终端：

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
ros2 topic echo /control/qr_mission_state
```

用 `python3 -m json.tool <文件>` 排查损坏 JSON，用 `file *.png` 检查图片。停止：

```bash
cd ~/px4_ros2_ws
./simulation/scripts/stop_qr_sitl.sh
```

本目录不证明树莓派/Coral/Pixhawk/相机标定/系留或装桨实飞。
