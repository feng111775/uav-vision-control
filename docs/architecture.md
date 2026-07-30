# 系统架构说明

## D题总集成正式约束

- 总集成分支：`integration/d-task-final`。
- 视觉稳定版提交 `a5978ef65ec004b79c7ca7cb41011951adfdfe85` 已纳入总分支。
- 正式树莓派工作空间：`/home/a-corn/px4_ros2_ws`。
- 正式硬件基线：Holybro Pixhawk 6C Mini、PX4 Release v1.16.0，提交
  `6ea3539157ca358c70a515878b77077af7d4611d`，目标 `px4_fmu-v6c_default`；
  ROS 2 Jazzy、`px4_msgs release/1.16`、Micro XRCE-DDS Agent v2.4.3。
- `src/real_practise` 仅用于独立起飞→上升→悬停→降落验证，不是完整比赛任务入口。
- `src/uav_control` 是唯一正式比赛控制包，统一承载任务管理、计划中的
  `mission_offboard_controller`、小车启动网关、视觉目标接入和投放节点。
- 只有 `mission_offboard_controller` 允许发布 `/fmu/in/*`；其他节点禁止直接 ARM、切换
  Offboard、发布 `TrajectorySetpoint`、`VehicleCommand` 或任何 `/fmu/in/*` 控制消息。
- 投放当前必须保持 dry-run；STM32—ESP32—树莓派小车无线启动链尚未并入真机任务。
- 当前状态：**SITL已验证、真机待验收**。禁止直接装桨测试。
- 红色辅助功能暂停，未进入正式主线。

当前总分支仍保留旧控制入口。旧 `mission_controller_node`、
`vision_offboard_controller` 和 `offboard_control` 的现有源码不得被解释为最终架构；
它们等待正式飞控分支同步，不能据此声称真机验证通过、真机可直接起飞或投放舵机已完成。

## 1. 当前系统架构

本工程由 OpenMV 端视觉程序、ROS 2 视觉处理包和 PX4 控制包组成。

```text
OpenMV H7 Plus
  │ USB CDC文本协议
  ↓
h7_bridge_node
  │ /vision/h7/detection
  ↓
target_filter_node
  │ /vision/h7/filtered_detection
  ↓
visual_servo_node
  │ /control/vision_velocity
  ↓
vision_offboard_controller
  │ /fmu/in/*
  ↓
PX4
```

Gazebo 仿真使用 `gazebo_red_target_detector_node` 替代
`h7_bridge_node`，无硬件测试可以使用 `fake_h7_node`。这三个节点均发布
`/vision/h7/detection`，同一时间只能选择一个作为原始视觉数据源。

目录职责：

```text
openmv_h7plus/       OpenMV Cam H7 Plus端程序
src/uav_vision/      视觉输入、目标滤波和视觉伺服
src/uav_control/     唯一正式比赛控制包（任务与PX4控制统一归属）
docs/                架构、开发和部署文档
```

正式总体目录约定如下；不存在的接口目录不凭空创建：

```text
openmv_h7plus/          OpenMV H7 Plus正式视觉程序
src/real_practise/      独立起降验证包，不是比赛总入口（由对应分支提供时保留）
src/uav_control/        唯一正式比赛任务控制包
src/uav_vision/         OpenMV串口桥、滤波、预测和视觉误差输出
src/pi_camera_vision/   如存在则为历史/SITL视觉工具
src/uav_interfaces/     仅在项目实际使用且已有分支时保留；当前不创建
```

当前没有独立的任务规划层。任务管理器、通用轨迹生成器和航点执行器尚未实现。

## 2. ROS 2节点关系

### `uav_vision`

| 节点 | 输入 | 输出 | 职责 |
| --- | --- | --- | --- |
| `h7_bridge_node` | H7Plus USB串口 | `/vision/h7/detection` | 解析并校验OpenMV文本协议 |
| `fake_h7_node` | 无 | `/vision/h7/detection` | 发布模拟检测数据 |
| `gazebo_red_target_detector_node` | `/camera/down/image_raw` | `/vision/h7/detection`、`/vision/gazebo/debug_image` | 检测Gazebo图像中的红色目标 |
| `target_filter_node` | `/vision/h7/detection` | `/vision/h7/filtered_detection` | 目标确认、丢失判断和指数滤波 |
| `visual_servo_node` | `/vision/h7/filtered_detection` | `/control/vision_velocity` | 将像素偏差转换为FLU机体水平速度 |

检测数据使用 `std_msgs/msg/Float32MultiArray`，字段顺序为：

```text
[valid, cx, cy, width, height, area, confidence]
```

`visual_servo_node` 输出 `geometry_msgs/msg/TwistStamped`，坐标系为
`base_link`，仅使用水平线速度。

### `uav_control`（当前旧入口审计）

| 节点 | 输入 | 输出 | 职责 |
| --- | --- | --- | --- |
| `vision_offboard_controller` | `/control/vision_velocity`、PX4位置和状态 | PX4 Offboard心跳、轨迹设定值和命令 | 旧控制入口，等待正式 `mission_offboard_controller` 分支同步 |
| `vehicle_status_listener` | 可配置的PX4状态topic | 日志 | 只读状态诊断 |
| `offboard_control` | 无 | PX4控制topic | 旧测试节点，违反单一发布者约束，禁止作为正式入口 |

当前源码审计发现 `mission_controller_node`、`vision_offboard_controller` 和
`offboard_control` 均包含 `/fmu/in/*` 发布代码；这属于旧入口待替换状态，不符合
新架构的单一发布者要求。正式飞控分支合入后，必须只保留
`mission_offboard_controller` 作为 `/fmu/in/*` 发布者。

## 3. PX4通信关系

`vision_offboard_controller` 订阅：

| Topic | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | 本地位置、heading和有效性 |
| `/fmu/out/vehicle_status` | `px4_msgs/msg/VehicleStatus` | 解锁、导航模式和failsafe |

这两个 topic 可以分别通过以下参数覆盖：

```text
vehicle_local_position_topic
vehicle_status_topic
```

控制器发布：

| Topic | 类型 | 用途 |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | Offboard控制模式心跳 |
| `/fmu/in/trajectory_setpoint` | `px4_msgs/msg/TrajectorySetpoint` | NED速度设定值 |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | 模式切换和仿真自动解锁 |

坐标约定：

- ROS视觉速度使用FLU：X向前、Y向左、Z向上。
- PX4本地控制使用NED：X向北、Y向东、Z向下。
- 控制器根据PX4 heading将FLU水平速度转换为NED速度。

ROS 2与PX4之间还需要独立运行的uXRCE-DDS通信链：

```text
ROS 2节点
  ↕ DDS
Micro XRCE-DDS Agent
  ↕ Serial或UDP
PX4 uxrce_dds_client
```

该Agent和PX4端通信参数不由当前launch文件启动或配置。

## 4. 各文件职责

### OpenMV

| 文件 | 职责 |
| --- | --- |
| `openmv_h7plus/main.py` | 相机检测和协议发送主循环 |
| `openmv_h7plus/camera_config.py` | CSI相机初始化 |
| `openmv_h7plus/detector.py` | 自适应黑色同心圆和中央十字检测 |
| `openmv_h7plus/protocol.py` | USB CDC D_TARGET与D_STATUS文本协议 |

### 视觉包

| 文件 | 职责 |
| --- | --- |
| `h7_bridge_node.py` | H7Plus串口桥 |
| `fake_h7_node.py` | 模拟检测输入 |
| `gazebo_red_target_detector_node.py` | Gazebo红色目标检测 |
| `target_filter_node.py` | 目标滤波状态机 |
| `visual_servo_node.py` | 像素误差到水平速度转换 |
| `gazebo_vision.launch.py` | Gazebo视觉链启动 |
| `gazebo_vision.yaml` | Gazebo检测、滤波和伺服参数 |

### 控制包

| 文件 | 职责 |
| --- | --- |
| `vision_offboard_controller.py` | 正式视觉Offboard状态机和PX4接口 |
| `vehicle_status_listener.py` | PX4状态只读诊断 |
| `offboard_control.py` | PX4基础通信测试 |
| `uav_control.launch.py` | 只启动正式控制器 |
| `control.yaml` | 正式控制器默认参数 |

### 构建和测试

| 文件或目录 | 职责 |
| --- | --- |
| `package.xml` | ROS 2包依赖和元数据 |
| `setup.py` | Python包、节点入口、launch和配置安装 |
| `setup.cfg` | ROS 2 Python脚本安装路径 |
| `test/` | 协议、滤波、视觉伺服和控制状态机测试 |

## 5. 当前版本能力

已经具备：

- OpenMV H7 Plus红色目标检测；
- H7Plus串口协议解析和断线重连；
- Gazebo红色目标检测；
- 无硬件模拟视觉输入；
- 目标置信度过滤、帧确认和指数平滑；
- 基于像素误差的视觉伺服速度；
- 视觉输入超时零速度保护；
- FLU到NED水平速度转换；
- PX4本地位置和状态检查；
- Offboard预流、模式请求和仿真自动解锁；
- 起飞至固定本地NED高度；
- 水平、垂直速度限制；
- PX4状态、位置和起飞超时保护；
- SITL视觉Offboard基础验证。

当前不具备：

- 通用任务规划；
- 航点序列；
- 通用位置轨迹生成；
- 自动任务降落；
- ROS命令ACK和重试；
- 完整真机启动链；
- 真机人工授权和接管接口；
- 明确的最终failsafe动作。

当前 `vision_offboard_controller` 在状态机入口要求
`simulation_mode=true`，因此不能通过简单修改YAML直接用于真机自主飞行。

## 6. 比赛后续扩展位置

建议在 `src/uav_control/uav_control/` 中增加独立任务层：

```text
mission_manager.py
trajectory_generator.py
```

推荐职责：

```text
mission_manager
  │ 比赛规则、任务阶段、航点和动作
  ↓
trajectory_generator
  │ 连续位置、速度、yaw参考
  ↓
控制接口/仲裁器
  ↓
vision_offboard_controller或后续通用控制节点
  ↓
PX4
```

比赛规则变化应主要发生在任务层，不应把题目流程直接写入PX4通信、坐标转换或
安全状态机。

扩展时需要先定义：

- 任务命令接口；
- 航点序列接口；
- 位置、速度和yaw参考接口；
- 启动、暂停、恢复、取消和降落接口；
- 任务进度与失败原因反馈；
- 任务轨迹、视觉伺服和failsafe之间的优先级。

无论如何扩展，都应保持单一PX4控制发布者，避免多个节点同时向
`/fmu/in/*`发送互相冲突的设定值。
