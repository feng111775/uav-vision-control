# UAV Vision Control

## 1. 项目简介

本工程实现无人机视觉目标检测、目标数据滤波、视觉伺服速度生成以及 PX4
Offboard 控制。工程同时包含 OpenMV Cam H7 Plus 端程序和 ROS 2 Python
节点，可使用以下两类视觉输入：

- 真实硬件：OpenMV H7 Plus 检测红色色块，通过 USB CDC 串口发送检测结果。
- Gazebo 仿真：ROS-Gazebo 图像桥接后，在 ROS 2 侧检测红色目标。

两类输入统一发布为 `/vision/h7/detection`，后续滤波、视觉伺服和飞控链路共用
同一接口。

目录结构如下：

```text
uav-vision-control/
├── openmv_h7plus/       # OpenMV H7 Plus MicroPython 程序
└── src/
    ├── uav_vision/      # 视觉输入、滤波和视觉伺服 ROS 2 包
    └── uav_control/     # PX4 状态监听及 Offboard 控制 ROS 2 包
```

## 2. 系统架构

真实硬件链路：

```text
OpenMV H7 Plus
  └─ USB CDC: TARGET,valid,cx,cy,width,height,area,confidence
       └─ h7_bridge_node
            └─ /vision/h7/detection
                 └─ target_filter_node
                      └─ /vision/h7/filtered_detection
                           └─ visual_servo_node
                                └─ /control/vision_velocity
                                     └─ vision_offboard_controller
                                          └─ PX4 /fmu/in/*
```

Gazebo 仿真链路：

```text
Gazebo 相机
  └─ ros_gz_bridge: /camera/down/image_raw
       └─ gazebo_red_target_detector_node
            └─ /vision/h7/detection
                 └─ target_filter_node
                      └─ visual_servo_node
                           └─ vision_offboard_controller
                                └─ PX4 SITL
```

真实 H7Plus 桥接节点与 Gazebo 检测节点是同一原始检测 topic 的两个替代数据源，
不应同时运行。

坐标约定：

- 视觉伺服输出使用 ROS `base_link` 的 FLU 坐标系：X 向前、Y 向左、Z 向上。
- PX4 本地控制使用 NED 坐标系：X 向北、Y 向东、Z 向下。
- `vision_offboard_controller` 根据 PX4 heading 将机体水平速度转换为 NED
  速度设定值。

## 3. 软件环境

工程依赖 ROS 2 的 `ament_python`/`colcon` 构建体系。具体 ROS 2、PX4 和
`px4_msgs` 版本应保持消息定义兼容。

主要依赖：

- ROS 2、Python 3、`colcon`
- ROS 2 包：`rclpy`、`std_msgs`、`geometry_msgs`、`sensor_msgs`
- PX4 ROS 2 消息包：`px4_msgs`
- 视觉组件：OpenCV、NumPy、`cv_bridge`
- Gazebo 联调：`ros_gz_bridge`、`rosgraph_msgs`、`launch_ros`
- H7Plus 联调：PySerial
- OpenMV Cam H7 Plus：OpenMV 固件 5.0 / MicroPython 1.28

使用真实飞控或 PX4 SITL 时，还需要启动与当前 PX4 版本匹配的 ROS 2/DDS
通信链路，并确认 `/fmu/in/*` 和 `/fmu/out/*` topic 已建立。

## 4. ROS 2 节点说明

### `uav_vision`

| 可执行节点 | 作用 | 主要输入 | 主要输出 |
| --- | --- | --- | --- |
| `h7_bridge_node` | 读取并校验 H7Plus 串口协议；断线后周期重连 | USB 串口，默认 `/dev/ttyACM0`、115200 | `/vision/h7/detection` |
| `fake_h7_node` | 以 10 Hz 发布模拟目标数据，用于无硬件测试 | 无 | `/vision/h7/detection` |
| `gazebo_red_target_detector_node` | 从 Gazebo 相机图像检测红色目标 | `/camera/down/image_raw`（可配置） | `/vision/h7/detection`、`/vision/gazebo/debug_image` |
| `target_filter_node` | 置信度判定、帧确认、丢失判定和平滑滤波 | `/vision/h7/detection` | `/vision/h7/filtered_detection` |
| `visual_servo_node` | 将像素偏差转换为 `base_link` 水平速度；输入超时发布零速度 | `/vision/h7/filtered_detection` | `/control/vision_velocity` |

检测消息使用 `std_msgs/msg/Float32MultiArray`，数据顺序为：

```text
[valid, cx, cy, width, height, area, confidence]
```

`valid` 为 0 或 1，`confidence` 范围为 0～100。

### `uav_control`

| 可执行节点 | 作用 | 说明 |
| --- | --- | --- |
| `vehicle_status_listener` | 订阅并打印 PX4 解锁、导航和 failsafe 状态 | 当前固定订阅 `/fmu/out/vehicle_status_v1` |
| `offboard_control` | 发布固定位置目标及 PX4 命令 | 示例节点会请求 Offboard、解锁，并在约 15 秒后请求降落 |
| `vision_offboard_controller` | 接收视觉速度和 PX4 状态，发布速度模式 Offboard 心跳、速度设定值及必要命令 | PX4 状态 topic 可通过参数配置；默认不启用 Offboard 和自动解锁 |

`vision_offboard_controller` 包含 `WAITING`、`PRESTREAM`、`TAKEOFF`、
`VISION_CONTROL` 和 `FAILSAFE` 状态。自动解锁只允许在
`simulation_mode=true`、`enable_offboard=true` 和
`enable_auto_arm=true` 同时设置时执行。

## 5. PX4 通信 topic 说明

| 方向 | Topic | 消息类型 | 用途 |
| --- | --- | --- | --- |
| ROS 2 → PX4 | `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Offboard 控制模式心跳 |
| ROS 2 → PX4 | `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | 位置或速度设定值 |
| ROS 2 → PX4 | `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | 模式切换、解锁、降落等命令 |
| PX4 → ROS 2 | `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | 本地位置、航向及有效性 |
| PX4 → ROS 2 | `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | 解锁、导航模式和 failsafe 状态 |

`vision_offboard_controller` 的 PX4 输出 topic 参数：

| 参数 | 默认值 |
| --- | --- |
| `vehicle_local_position_topic` | `/fmu/out/vehicle_local_position` |
| `vehicle_status_topic` | `/fmu/out/vehicle_status` |

如果当前 PX4 消息桥使用 `_v1` 后缀，可在启动节点时覆盖：

```bash
ros2 run uav_control vision_offboard_controller --ros-args \
  -p vehicle_local_position_topic:=/fmu/out/vehicle_local_position_v1 \
  -p vehicle_status_topic:=/fmu/out/vehicle_status_v1
```

可通过以下命令检查实际 topic：

```bash
ros2 topic list | grep '^/fmu/'
ros2 topic info /fmu/out/vehicle_status
```

## 6. 编译方法

在工程根目录执行：

```bash
cd ~/uav-vision-control
source /opt/ros/<ros_distro>/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

将 `<ros_distro>` 替换为当前安装的 ROS 2 发行版名称。每个新终端都需要重新
加载 ROS 2 和本工作空间环境。

运行测试：

```bash
cd ~/uav-vision-control
source /opt/ros/<ros_distro>/setup.bash
source install/setup.bash
colcon test
colcon test-result --verbose
```

## 7. 运行方法

### 7.1 H7Plus 实机视觉链路

先按照 `openmv_h7plus/README.md` 将程序上传到开发板，并确保 OpenMV IDE
未占用 USB CDC 串口。

分别在终端中运行：

```bash
source install/setup.bash
ros2 run uav_vision h7_bridge_node --ros-args \
  -p port:=/dev/ttyACM0 -p baudrate:=115200
```

```bash
source install/setup.bash
ros2 run uav_vision target_filter_node
```

```bash
source install/setup.bash
ros2 run uav_vision visual_servo_node
```

无 H7Plus 时，可用以下节点替代 `h7_bridge_node`：

```bash
source install/setup.bash
ros2 run uav_vision fake_h7_node
```

### 7.2 Gazebo 视觉链路

在 Gazebo 相机和 ROS-Gazebo 通信环境已启动后执行：

```bash
source install/setup.bash
ros2 launch uav_vision gazebo_vision.launch.py
```

该 launch 文件启动图像桥、Gazebo 红色目标检测、目标滤波和视觉伺服，不启动
PX4 控制器，也不会解锁无人机。参数位于
`src/uav_vision/config/gazebo_vision.yaml`。

### 7.3 PX4 视觉 Offboard 控制

先启动 PX4 SITL、对应的 ROS 2/DDS 通信链路和上述任一视觉链路。确认以下
topic 持续更新：

```bash
ros2 topic hz /control/vision_velocity
ros2 topic hz /fmu/out/vehicle_local_position
ros2 topic hz /fmu/out/vehicle_status
```

仅观察控制器输出、不请求 Offboard 或解锁：

```bash
source install/setup.bash
ros2 run uav_control vision_offboard_controller
```

在 PX4 SITL 中显式启用 Offboard 和自动解锁：

```bash
source install/setup.bash
ros2 run uav_control vision_offboard_controller --ros-args \
  -p simulation_mode:=true \
  -p enable_offboard:=true \
  -p enable_auto_arm:=true
```

运行前应确认仿真环境、坐标方向、目标高度、速度限制、PX4 状态反馈和紧急停止
方式均符合预期。不要将仅为 SITL 设计的自动解锁参数直接用于真实飞行器。

独立状态观察：

```bash
source install/setup.bash
ros2 run uav_control vehicle_status_listener
```

该节点当前使用 `_v1` 状态 topic；应先确认它与所运行 PX4 版本一致。

## 8. 后续开发说明

- 将 ROS 2 发行版、PX4 版本、`px4_msgs` 分支和 Gazebo 版本固定到可复现的
  开发环境配置中。
- 为 `uav_control` 增加 launch 文件和 YAML 参数文件，统一仿真、台架和实机
  配置。
- 统一所有 PX4 状态节点的 topic 参数化方式，避免默认名称与 `_v1` 后缀并存。
- 根据真实相机安装方向、视场角和飞行高度标定视觉伺服比例、符号、死区及限速。
- 补充串口协议版本、消息时间戳和链路状态诊断，区分目标丢失、数据超时与设备
  断开。
- 在真实飞行前增加硬件在环测试、控制权限隔离、人工接管、地理围栏和降落策略
  验证。
- 保持视觉处理逻辑与 ROS 2 接口分离，并为协议解析、滤波、坐标转换、状态机和
  failsafe 路径持续补充自动化测试。
