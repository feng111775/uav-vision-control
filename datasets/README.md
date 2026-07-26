# QR 数据集工具与边界

本目录用于生成和检查离线 QR 检测训练集；它不参与正式 SITL 盘点。正式
`qr_detector_node` 只解码 `/camera/front/image_raw` 当前帧，绝不从本目录
读取 QR PNG 作为任务答案。`samples/` 是可提交的算法样例，完整生成数据被
`.gitignore` 排除。

## 从零准备

支持 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。首次获取代码：

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
```

目录内容：

```text
datasets/
├── README.md
└── samples/       # 小型、可提交的解码回归样例
```

生成固定种子 `20260726` 的隔离 train/val/test 变体：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
python3 src/uav_vision/tools/generate_qr_dataset.py \
  --output datasets/qr_shelf --train 160 --val 48 --test 48
python3 -m pytest -q src/uav_vision/test/test_qr_core.py
```

该工具没有 ROS 话题。输出是图像、标注和统计文件；参数
`--train/--val/--test` 控制各集合数量。成功标准是命令返回 0、三个集合非空，
且 QR 1～24 解码单测通过。

常见故障：

```bash
cd ~/px4_ros2_ws
python3 -c "import cv2,numpy; print(cv2.__version__)"
git status --ignored --short datasets
```

生成任务可用 `Ctrl-C` 停止，删除 `datasets/qr_shelf` 后可重新生成。该离线
数据结果不等于 Gazebo 相机验证，更不等于树莓派、Coral 或真机飞行验证。
