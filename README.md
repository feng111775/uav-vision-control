# H2025 Wildlife Inspection UAV System

本项目是面向 2025 年全国大学生电子设计竞赛 H 题的无人机野生动物巡检仿真系统。系统基于以下组件构建：

- PX4 Autopilot
- ROS 2 Jazzy
- Gazebo Harmonic
- Python ROS 2 节点
- MicroXRCE DDS
- ROS 2 + PX4 Offboard 控制

当前 H2025 版本已经实现：

- PX4 SITL 无人机仿真
- Gazebo `wildlife_inspection` 野生动物巡检环境
- 自动起飞
- 9×7 网格覆盖巡航
- 自动返航
- 自动降落
- 禁飞区域规划框架
- UAV 实时位置和任务路线可视化
- PX4 local NED 与 Gazebo world ENU 坐标转换

本文面向第一次接触 PX4、ROS 2 和 Gazebo 的使用者。建议严格按照终端编号依次启动，先确认每一步正常，再继续下一步。

---

# 1. 项目结构

与 H2025 任务直接相关的主要目录如下：

```text
uav-vision-control/
├── src/
│   ├── mission_interfaces/                 # 任务自定义ROS 2消息
│   └── uav_control/
│       └── uav_control/
│           ├── mission_manager.py          # MissionState任务状态机
│           ├── mission_manager_node.py     # 任务规划和任务接口
│           ├── mission_offboard_controller.py
│           │                               # PX4 Offboard位置控制
│           ├── px4_position_bridge_node.py # PX4 NED位置反馈转换
│           ├── coverage_planner.py         # 覆盖路径和禁飞区绕行
│           └── mission_visualizer.py       # 地图、路线和UAV位置显示
├── simulation/
│   └── px4_overlay/
│       └── Tools/simulation/gz/
│           ├── worlds/
│           │   └── wildlife_inspection.sdf
│           └── models/
│               └── wildlife/
└── README.md
```

`simulation/px4_overlay` 保存本任务使用的 Gazebo 世界和动物模型资源。运行仿真时，对应资源需要位于 PX4 工程的 `Tools/simulation/gz` 路径中。

---

# 2. 环境要求

当前验证通过的环境：

| 组件 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| PX4 Autopilot | v1.15.4 |
| Gazebo | Harmonic |
| Python | 3.12 |
| PX4/ROS 2通信 | MicroXRCE DDS |

推荐使用 16 GB 或更多内存。PX4 SITL、Gazebo、ROS 2 节点和 Matplotlib 可视化同时运行时会占用较多内存和 CPU。

开始前应确认以下工程目录存在：

```text
~/2025h/uav-vision-control
~/2025h/PX4-Autopilot
```

---

# 3. 工程编译

进入 ROS 2 工程：

```bash
cd ~/2025h/uav-vision-control
```

加载 ROS 2 Jazzy：

```bash
source /opt/ros/jazzy/setup.bash
```

编译任务消息和控制包：

```bash
colcon build \
  --packages-select mission_interfaces uav_control \
  --symlink-install
```

加载本工作空间：

```bash
source install/setup.bash
```

检查 `uav_control` 可执行节点：

```bash
ros2 pkg executables uav_control
```

如果修改过 Python 节点，使用 `--symlink-install` 后通常无需重复复制源码，但新终端仍必须重新执行两条 `source` 命令。

---

# 4. 完整启动流程（重点）

以下命令需要在 8 个终端中分别运行。除检查终端外，节点启动后都应保持窗口运行。

## 终端1：启动MicroXRCE DDS

加载 ROS 2：

```bash
source /opt/ros/jazzy/setup.bash
```

启动 Agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

该窗口负责 PX4 与 ROS 2 DDS 网络通信，启动后不要关闭。

---

## 终端2：启动PX4 + Gazebo wildlife环境

进入 PX4 工程：

```bash
cd ~/2025h/PX4-Autopilot
```

启动 PX4 SITL、下视相机机型和 wildlife 世界：

```bash
PX4_GZ_MODEL_POSE="20,-15,2,0,0,0" \
PX4_GZ_WORLD=wildlife_inspection \
make px4_sitl gz_x500_downward_camera
```

等待 PX4 控制台出现：

```text
Ready for takeoff!
```

不要关闭 PX4 或 Gazebo 窗口。

无人机初始 Gazebo world ENU 位置为：

```text
x = 20
y = -15
z = 2
```

---

## 终端3：检查PX4 ROS2通信

加载 ROS 2：

```bash
source /opt/ros/jazzy/setup.bash
```

检查 PX4 topic：

```bash
ros2 topic list | grep fmu
```

正常情况下至少应出现：

```text
/fmu/out/vehicle_local_position
/fmu/out/vehicle_status
```

如果没有输出，先检查终端1中的 MicroXRCEAgent 和终端2中的 PX4 是否仍在运行。

---

## 终端4：启动PX4位置转换节点

进入工程并加载环境：

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

启动位置桥：

```bash
ros2 run uav_control px4_position_bridge_node \
  --ros-args \
  -p origin_north:=15.0 \
  -p origin_east:=-20.0 \
  -p origin_down:=0.0
```

该节点将 `/fmu/out/vehicle_local_position` 中的 PX4 local NED 坐标转换为任务使用的 Gazebo world ENU 坐标，并发布到：

```text
/mission/current_position
```

---

## 终端5：启动任务管理节点

进入工程并加载环境：

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

启动任务管理器：

```bash
ros2 run uav_control mission_manager_node
```

正常启动后会看到类似信息：

```text
任务管理节点已启动
任务状态：IDLE
```

该节点负责生成覆盖路线、处理禁飞区域、发布当前任务目标并推进任务状态机。

---

## 终端6：启动Offboard控制

进入工程并加载环境：

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

启动任务 Offboard 控制器：

```bash
ros2 run uav_control mission_offboard_controller \
  --ros-args \
  -p origin_north:=15.0 \
  -p origin_east:=-20.0 \
  -p origin_down:=0.0
```

Offboard 控制器将任务 ENU 目标转换为 PX4 local NED 位置设定值，并负责 Offboard 模式、解锁、起飞保持和自动降落控制。

终端4和终端6的三个 `origin` 参数必须完全一致。

---

## 终端7：启动任务可视化

进入工程并加载环境：

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

启动可视化节点：

```bash
ros2 run uav_control mission_visualizer
```

窗口显示：

- 45 m × 35 m任务地图
- 绿色起飞点
- 橙色UAV实时位置
- 实际任务目标路线
- 禁飞区域

实际任务路线来自 `/mission/trajectory_point`。该 topic 发布当前任务目标，因此路线会随任务执行逐步显示。

---

## 终端8：开始任务

加载 ROS 2 和工程环境：

```bash
source /opt/ros/jazzy/setup.bash
source ~/2025h/uav-vision-control/install/setup.bash
```

调用任务启动服务：

```bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

任务状态按照以下顺序推进：

```text
IDLE
  ↓
TAKEOFF
  ↓
EXECUTE
  ↓
RETURN
  ↓
LAND
  ↓
COMPLETE
```

任务启动后不要重复调用 `/mission/start`。需要观察状态时，可以在新的终端执行：

```bash
ros2 topic echo /mission/status
```

---

# 5. 当前任务参数

| 参数 | 当前值 |
| --- | --- |
| 地图尺寸 | 45 m × 35 m |
| 网格数量 | 9 × 7 |
| 单格尺寸 | 5 m × 5 m |
| Gazebo起飞点 | `(20, -15, 2)` |
| 第一覆盖点 | `(-20, -15, 2)` |
| 返航点 | `(20, -15, 2)` |
| 巡航速度 | 5 m/s |
| 禁飞格 | `A3B2`、`A3B3`、`A3B4` |

覆盖规划器会跳过禁飞格，并为可能穿过禁飞区域的航段生成绕行点。

---

# 6. 坐标系统说明（重要）

系统中存在两套实际坐标系。

## Gazebo world / 任务地图：ENU

```text
x = East，向东为正
y = North，向北为正
z = Up，向上为正
```

任务规划、禁飞区、起飞点、返航点和可视化均使用 Gazebo world ENU 绝对坐标。

## PX4 local：NED

```text
x = North，向北为正
y = East，向东为正
z = Down，向下为正
```

PX4 local NED 的水平原点位于无人机出生位置，而本任务的 Gazebo world ENU 原点位于地图中心。无人机出生在：

```text
Gazebo ENU = (20, -15, 2)
```

因此，ENU 与NED轴交换后还需要补偿水平原点偏移：

```text
origin_north = 15
origin_east  = -20
origin_down  = 0
```

任务坐标到 PX4 坐标的转换为：

```text
north = origin_north + gazebo_y
east  = origin_east  + gazebo_x
down  = origin_down  - gazebo_z
```

例如起飞目标：

```text
Gazebo ENU (20, -15, 2)
        ↓
PX4 NED    (0, 0, -2)
```

反向位置桥使用相同的 origin，把 PX4 local NED 位置恢复成 Gazebo world ENU。两个转换节点的 origin 不一致时，飞机位置显示、到达判定和飞行目标都会出现整体偏移。

---

# 7. 常见问题

## ros2 run提示Package not found

通常是当前终端没有加载编译后的工作空间。

解决：

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

然后重新执行 `ros2 run`。

## PX4提示already running

说明上一次 PX4 进程没有正常退出。

解决：

```bash
pkill px4
```

确认旧进程结束后，回到 PX4 工程重新启动。

## Gazebo关闭异常

如果 Gazebo 窗口关闭后后台进程仍然存在，可执行：

```bash
pkill gz
```

该命令会结束当前用户的同名 Gazebo 进程，执行前应确认没有其他需要保留的 Gazebo 仿真。

## 飞机飞出地图

首先检查位置桥和 Offboard 控制器是否使用了完全相同的参数：

```text
origin_north = 15
origin_east  = -20
origin_down  = 0
```

需要检查的两个节点：

```text
px4_position_bridge_node
mission_offboard_controller
```

如果任一节点仍使用零偏移，Gazebo绝对目标会被错误解释成相对出生点的 PX4 目标，飞机可能飞向地图外。

## 看不到UAV实时位置

检查 PX4位置反馈和转换后的任务位置：

```bash
ros2 topic echo /fmu/out/vehicle_local_position --once
ros2 topic echo /mission/current_position --once
```

无人机位于出生点附近时，`/mission/current_position` 的水平坐标应接近：

```text
x = 20
y = -15
```

## 可视化路线刚启动时不完整

`mission_visualizer` 订阅的是当前任务目标 `/mission/trajectory_point`，并从启动后开始累积路线。为了看到完整执行过程，应在调用 `/mission/start` 之前启动可视化节点。

---

# 8. 后续开发方向

以下内容是未来规划，当前 H2025 v1.0 不声称已经完成。

## 1. 基于摄像头自动识别禁飞区域

计划流程：

```text
camera
  ↓
image processing
  ↓
obstacle detection
  ↓
no_fly_cells
  ↓
coverage planner
```

目标是将图像检测结果转换为网格禁飞区，再交给现有覆盖规划器重新生成安全路线。

## 2. 动物识别统计

计划流程：

```text
camera
  ↓
YOLO
  ↓
animal classification
  ↓
statistics
```

目标是识别不同动物类别、记录位置并生成巡检统计结果。

---

# 9. 版本信息

当前稳定版本：

```text
H2025 v1.0
```

Git tag：

```text
h2025-v1.0
```
