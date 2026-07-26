# UAV Vision Control

## 从零复现二维码逐码扫描 GUI SITL

已验证组合：Ubuntu 24.04、ROS 2 Jazzy、PX4 v1.17.0、Gazebo Sim 8。
Gazebo 主窗口是第三人称视角；`rqt_image_view` 才显示无人机真实前视相机、
解码框和逐码状态。

### 1. 安装依赖

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
export ROS_APT_SOURCE_VERSION="$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | cut -d '"' -f 4)"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y git curl gnupg lsb-release build-essential cmake \
  python3-pip python3-venv python3-opencv python3-numpy \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-desktop ros-jazzy-ros-gz-bridge \
  ros-jazzy-rqt-image-view ros-jazzy-cv-bridge
sudo rosdep init 2>/dev/null || true
rosdep update
```

安装 MicroXRCEAgent：

```bash
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git \
  ~/Micro-XRCE-DDS-Agent
cd ~/Micro-XRCE-DDS-Agent
mkdir -p build
cd ~/Micro-XRCE-DDS-Agent/build
cmake ..
make -j"$(nproc)"
sudo make install
sudo ldconfig
MicroXRCEAgent --version
```

### 2. 下载正确分支和 PX4

```bash
git clone -b feature/qr-shelf-sitl \
  https://github.com/feng111775/uav-vision-control.git \
  ~/px4_ros2_ws
git clone --recursive -b v1.17.0 \
  https://github.com/PX4/PX4-Autopilot.git \
  ~/PX4-Autopilot
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
git submodule update --init --recursive
make px4_sitl_default
```

### 3. 构建与测试

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source ~/px4_ros2_ws/install/setup.bash
git diff --check
python3 -m compileall src
bash -n simulation/scripts/*.sh
colcon test
colcon test-result --verbose
```

仓库忽略本地 `src/px4_msgs`；若系统没有匹配消息包，需先放入与 PX4 v1.17
兼容的 `px4_msgs` 源码再构建。

### 4. 生成、安装并验证 overlay

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
python3 simulation/scripts/generate_qr_shelf_assets.py
./simulation/scripts/install_px4_overlay.sh ~/PX4-Autopilot
./simulation/scripts/install_px4_overlay.sh ~/PX4-Autopilot --apply
test -f ~/PX4-Autopilot/Tools/simulation/gz/worlds/qr_shelf_world.sdf
test -f ~/PX4-Autopilot/Tools/simulation/gz/models/qr_shelf/model.sdf
test -f ~/PX4-Autopilot/Tools/simulation/gz/models/x500_downward_camera/model.sdf
cd ~/PX4-Autopilot
make px4_sitl_default
```

### 5. 一键启动

确认没有连接真实 Pixhawk：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
TARGET_QR_ID=10 ./simulation/scripts/run_qr_sitl_gui.sh
```

脚本检查 Jazzy、PX4 v1.17、Agent 和 overlay，清理冲突实例，启动唯一 Agent、
`qr_shelf_world` GUI、PX4、`ros_gz_bridge`、QR 检测器和唯一正式控制器
`vision_offboard_controller.py`。`TARGET_QR_ID` 支持 1～24。

### 6. 观察真实识别、盘点和任务状态

真实识别调试画面：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /vision/qr/debug_image
```

盘点 JSON：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo /vision/qr/inventory
```

任务状态：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo /control/qr_mission_state
```

下视原始画面：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /camera/down/image_raw
```

### 7. 手动四终端流程

终端 1：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
MicroXRCEAgent udp4 -p 8888
```

终端 2：

```bash
cd ~/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
PX4_GZ_WORLD=qr_shelf_world make px4_sitl gz_x500_downward_camera
```

终端 3：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 launch uav_vision qr_shelf_task.launch.py mode:=sitl \
  inventory_mode:=full target_qr_id:=10 simulation_mode:=true \
  enable_offboard:=true enable_auto_arm:=true use_sim_time:=true
```

终端 4：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view /vision/qr/debug_image
```

## 真实二维码数据链与防作弊边界

```text
Gazebo front camera sensor
  → /camera/front/image (gz.msgs.Image)
  → ros_gz_bridge
  → /camera/front/image_raw (sensor_msgs/msg/Image)
  → CvBridge 当前帧
  → OpenCV QRCodeDetector.detectAndDecodeMulti()
  → 解码文本严格解析为 1..24
  → QRObservation.qr_id
  → QRInventory 当前期望 ID + 连续 confirm_frames 帧
  → /vision/qr/inventory JSON
  → QRMission
```

扫描计划只提供航点、扫描序号和“当前期望 QR ID”，不能提供实际识别结果。
没有图像、空白图像、解码失败、错误 ID 或非当前 ID 都不能推进。扫描位置到达
也不能确认；`scan_index` 不会生成 `QRObservation`。程序不读取货架 QR PNG
回答任务，模型权重只可定位候选框，编号仍由当前相机像素解码。

蛇形顺序：

```text
1 → 2 → 3 → 4 → 5 → 6
12 → 11 → 10 → 9 → 8 → 7
13 → 14 → 15 → 16 → 17 → 18
24 → 23 → 22 → 21 → 20 → 19
```

每点必须先 `QR_SCAN_MOVE → QR_SCAN_HOLD → QR_SCAN_CONFIRM`，然后连续
`confirm_frames` 帧实际解码为期望 ID 才进入 `QR_SCAN_NEXT`。非当前 ID 只写入
`visible_ids`。超时执行有限小范围搜索，重试耗尽进入 FAILSAFE，不会跳过。
只有真实 24/24 后才能进入 `TARGET_ACQUIRE` 并选择 `target_qr_id`。

### 遮挡验证

在 GUI SITL 到达当前扫描点前，用 Gazebo GUI 插入不透明 box 挡住当前二维码，
或临时移动该码的 visual；不要修改 ROS 话题或测试接口。观察：

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic echo /vision/qr/inventory
```

预期确认数保持不变，debug 显示 RETRY，最终可能 FAILSAFE。移除遮挡后，计数
仍不能立即跳变，必须重新积累连续 `confirm_frames` 帧。若位置一到就确认，
或遮挡后仍继续 24/24，即为验收失败。

## 目录、参数和接口

```text
src/uav_vision/       # 相机桥接后的 QR/红区检测、相机选择、视觉伺服
src/uav_control/      # PX4 接口和任务状态机
simulation/           # Gazebo/PX4 overlay 与启停脚本
datasets/             # 离线生成数据，不参与正式任务答案
models/               # 模型评估；模型不生成 QR ID
openmv_h7plus/        # 真机候选输入，未用于当前 SITL
docs/results/         # 小型验收证据
```

主要参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `target_qr_id` | 10 | 24/24 后的最终目标 |
| `confirm_frames` | 3 | 当前 ID 连续解码帧数 |
| `scan_timeout` | 8.0 s | 单次确认窗口 |
| `scan_max_retries` | 2 | 有限搜索重试次数 |
| `simulation_mode` | true（SITL launch） | 仿真安全门 |
| `enable_offboard` | true（一键 SITL） | 允许正式控制器输出 |
| `enable_auto_arm` | true（一键 SITL） | 仅限 SITL 自动解锁 |

主要话题：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/camera/front/image_raw` | `sensor_msgs/msg/Image` | Gazebo 前视真实图像 |
| `/camera/down/image_raw` | `sensor_msgs/msg/Image` | Gazebo 下视真实图像 |
| `/vision/qr/debug_image` | `sensor_msgs/msg/Image` | 解码框与盘点状态 |
| `/vision/qr/inventory` | `std_msgs/msg/String` | inventory JSON |
| `/control/qr_mission_state` | `std_msgs/msg/String` | 状态/扫描点 JSON |
| `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` | PX4 状态 |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/msg/VehicleLocalPosition` | PX4 位置 |

## 成功标准与排查

```bash
cd ~/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/px4_ros2_ws/install/setup.bash
ros2 topic hz /camera/front/image_raw
ros2 topic hz /camera/down/image_raw
ros2 topic hz /vision/qr/debug_image
ros2 topic echo /fmu/out/vehicle_status_v1 --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

成功日志必须从 QR 1 独立确认至 QR 19，共 24 项，然后才选择 QR 10，完成
前视接近、激光确认、切换 down、红区对准、PX4 Land、`landed=true`、
Disarmed、`failsafe=false`。

常见排查：

```bash
gz topic -l | grep /camera
ros2 topic info -v /camera/front/image_raw
ros2 topic echo /vision/qr/diagnostics
pgrep -af 'px4|gz sim|MicroXRCEAgent|vision_offboard_controller'
ss -lunp | grep ':8888'
```

## 停止与安全边界

```bash
cd ~/px4_ros2_ws
./simulation/scripts/stop_qr_sitl.sh
ss -lunp | grep ':8888' || echo "UDP 8888 released"
```

GUI SITL 的 `simulation_mode=true enable_offboard=true enable_auto_arm=true`
绝不能用于真实 Pixhawk。真机必须保持 `simulation_mode=false` 和
`enable_auto_arm=false`，并完成拆桨台架、真实相机/激光标定、遥控接管、地理
围栏、失联保护、系留和装桨实飞验证。树莓派 4B、Coral、Pixhawk、MTF-01、
真实双摄和激光全链路目前尚未完成验证。
