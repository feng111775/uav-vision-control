# QR detector 模型、训练与运行边界

本目录保存模型评估图和指标；正式小型 HOG+SVM 权重位于
`src/uav_vision/models/qr_hog_svm.xml`。模型只定位当前摄像头帧中的候选区域，
QR 编号仍必须由 OpenCV `QRCodeDetector` 解码文本得到，模型、航点和
`scan_index` 都不能生成编号。

## 环境与构建

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
python3 -m pytest -q src/uav_vision/test/test_qr_core.py
```

训练与评估：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
python3 src/uav_vision/training/train_hog_svm.py
python3 src/uav_vision/training/evaluate_backends.py
```

输入是 64×64 灰度 patch，输出是候选区域 decision score。ROS 参数
`detector_backend` 支持 `opencv`、`model`、`hybrid`；`model_path` 指向 XML，
`confidence_threshold` 过滤候选。正式话题为：

| 方向 | 话题 | 类型 |
|---|---|---|
| 输入 | `/camera/front/image_raw` | `sensor_msgs/msg/Image` |
| 输出 | `/vision/qr/detection` | `std_msgs/msg/Float32MultiArray` |
| 输出 | `/vision/qr/inventory` | `std_msgs/msg/String` JSON |
| 输出 | `/vision/qr/debug_image` | `sensor_msgs/msg/Image` |

单独观察节点：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 launch uav_vision qr_shelf_task.launch.py mode:=observe \
  detector_backend:=opencv use_sim_time:=true
```

没有相机发布者时 inventory 不会确认，属于正确行为。用
`ros2 topic hz /camera/front/image_raw` 和
`ros2 topic echo /vision/qr/diagnostics` 排查。用 `Ctrl-C` 停止观察节点。
当前 SVM 是 CPU 基线，不是标准目标检测器，没有 mAP50/mAP50-95，也未完成
Coral、树莓派实时性或真机相机验证。
