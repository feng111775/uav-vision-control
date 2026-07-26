# 2025 H题无人机野生动物巡查系统（Gazebo练习版）

## 1. 项目简介

本项目基于 PX4 SITL、Gazebo Harmonic 和 ROS 2 Jazzy，实现无人机自主巡查系统模拟。

项目目标是模拟 2025 年全国大学生电子设计竞赛 H 题野生动物巡查任务，包括：

- 区域覆盖规划
- 禁飞区域绕行
- 自动起飞
- Offboard 自主控制
- 动物检测统计模拟

> **说明：** 当前版本用于 2025 H 题训练，不影响正式比赛代码。

---

## 2. 软件环境

| 项目 | 版本或软件 |
| --- | --- |
| 系统 | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| 飞控 | PX4 v1.15.4 |
| 仿真 | Gazebo Harmonic |
| 通信 | MicroXRCEAgent |
| Python | Python 3 |

---

## 3. 系统架构

```text
Gazebo x500
      |
  PX4 SITL
      |
MicroXRCEAgent
      |
  ROS 2 Jazzy
      |
+-----------------+
| mission_manager |
+-----------------+
      |
      |
mission_offboard_controller
      |
      |
 PX4 Offboard
```

---

## 4. 工程目录

```text
src/
├── mission_interfaces/
│   └── 自定义 ROS 消息
│
├── uav_control/
│   ├── 任务管理
│   ├── 覆盖规划
│   ├── PX4 控制
│   ├── 可视化
│   └── 动物统计
│
└── uav_vision/
    └── 视觉相关模块
```

---

# 5. 完整运行流程

> **注意：必须按照以下终端顺序启动。**

## Terminal 1

启动 PX4 Gazebo：

```bash
cd ~/2025h/PX4-Autopilot
make px4_sitl gz_x500
```

等待终端出现：

```text
pxh>
```

出现 `pxh>` 表示 PX4 启动完成。

---

## Terminal 2

启动 DDS 通信：

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

看到以下信息：

```text
session established
```

表示通信成功。

---

## Terminal 3

进入 ROS 工程：

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

启动位置桥：

```bash
ros2 run uav_control px4_position_bridge_node
```

---

## Terminal 4

启动任务管理：

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uav_control mission_manager_node
```

---

## Terminal 5

启动 PX4 Offboard 控制：

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run uav_control mission_offboard_controller
```

---

## Terminal 6

启动任务：

```bash
source /opt/ros/jazzy/setup.bash
source ~/2025h/uav-vision-control/install/setup.bash

ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

---

# 6. 状态检查

查看 PX4 状态：

```bash
ros2 topic echo /fmu/out/vehicle_status --once
```

查看位置：

```bash
ros2 topic echo /fmu/out/vehicle_local_position --once
```

查看任务：

```bash
ros2 topic echo /mission/status
```

查看轨迹：

```bash
ros2 topic echo /fmu/in/trajectory_setpoint --once
```

查看 Offboard 发布频率：

```bash
ros2 topic hz /fmu/in/offboard_control_mode
```

---

# 7. 地图可视化

启动：

```bash
ros2 run uav_control mission_visualizer
```

地图可视化显示：

- 9×7 巡查区域
- 禁飞区
- 覆盖航线
- 当前无人机位置

---

# 8. 动物检测模拟

分别启动动物统计节点和动物检测模拟节点：

```bash
ros2 run uav_control animal_statistics_node
```

```bash
ros2 run uav_control animal_detector_sim_node
```

查看动物统计：

```bash
ros2 topic echo /animal/statistics
```

---

# 9. 当前完成内容

- ✓ Gazebo 无人机仿真
- ✓ PX4 Offboard 控制
- ✓ 自动模式切换
- ✓ 自动 Arm
- ✓ 起飞保持
- ✓ 覆盖航线生成
- ✓ 禁飞区绕行
- ✓ 任务状态管理
- ✓ 地图显示
- ✓ 动物检测模拟

---

# 10. 当前不足

- 动物识别仍为模拟数据
- 未连接真实无人机
- 未加入真实摄像头
- 未完成激光模块
- 未完成真实地面站硬件

---

# 11. Git版本

Branch：

```text
h2025-practice
```

Tag：

```text
v0.1-h2025-gazebo
```

该版本为 2025 H 题训练稳定版本。

---

# 12. Gazebo飞行与轨迹图生成

默认起飞高度和巡查高度均已调高到 2.0 m，便于在 Gazebo 中观察飞行过程。完整飞行与轨迹记录必须严格按照以下终端顺序启动。

## 一、启动顺序

### 终端1：启动PX4 Gazebo

```bash
cd /home/a-corn/2025h/PX4-Autopilot
make px4_sitl gz_x500
```

等待 PX4 终端出现 `pxh>` 后，再继续启动后续终端。

### 终端2：启动DDS通信

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

看到 `session established` 表示通信成功。

### 终端3：启动位置桥

```bash
cd /home/a-corn/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uav_control px4_position_bridge_node
```

### 终端4：启动任务管理

```bash
cd /home/a-corn/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uav_control mission_manager_node
```

### 终端5：启动PX4 Offboard控制

```bash
cd /home/a-corn/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uav_control mission_offboard_controller
```

### 终端6：启动飞行轨迹记录

```bash
cd /home/a-corn/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uav_control flight_trajectory_logger
```

### 终端7：启动任务

```bash
source /opt/ros/jazzy/setup.bash
source /home/a-corn/2025h/uav-vision-control/install/setup.bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

## 二、任务检查命令

查看任务状态：

```bash
ros2 topic echo /mission/status
```

查看 PX4 状态：

```bash
ros2 topic echo /fmu/out/vehicle_status --once
```

查看 Offboard 心跳频率：

```bash
ros2 topic hz /fmu/in/offboard_control_mode
```

查看轨迹设定值频率：

```bash
ros2 topic hz /fmu/in/trajectory_setpoint
```

## 三、轨迹图输出位置

任务完成后，轨迹图和 CSV 文件自动保存到：

```text
/home/a-corn/2025h/uav-vision-control/log/
```

输出文件名格式：

```text
h2025_flight_xy_<timestamp>.png
h2025_flight_actual_<timestamp>.csv
h2025_flight_planned_<timestamp>.csv
```

PNG 包含计划轨迹与实际轨迹的 XY 平面对比图，以及计划高度与实际高度曲线。

## 四、说明

- 默认起飞高度为 2.0 m。
- 默认巡查高度为 2.0 m。
- 任务进入 `COMPLETE` 后会自动生成飞行轨迹图和 CSV。
- 若中途按 `Ctrl+C` 结束轨迹记录节点，只要已有数据，也会自动保存。
- 如需调整高度，任务管理节点和 Offboard 控制节点必须使用相同的 `takeoff_height` 参数。

例如，将起飞高度和巡查高度都设置为 2.5 m：

```bash
ros2 run uav_control mission_manager_node --ros-args \
  -p takeoff_height:=2.5 -p patrol_height:=2.5
```

```bash
ros2 run uav_control mission_offboard_controller --ros-args \
  -p takeoff_height:=2.5
```

---

# 13. Demo模式与完整模式

## Demo模式运行方法

Demo 模式用于 Gazebo 快速展示。该模式保持自动起飞、Offboard 控制、RETURN 和 LAND 流程不变，只加载完整 coverage trajectory 的前 20 个轨迹点，以缩短演示时间。

终端 4 启动任务管理节点时加载 Demo 配置：

```bash
cd /home/a-corn/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uav_control mission_manager_node --ros-args \
  --params-file src/uav_control/config/demo_mode.yaml
```

其他终端仍按照“Gazebo飞行与轨迹图生成”章节中的顺序启动。最后在终端 7 调用：

```bash
source /opt/ros/jazzy/setup.bash
source /home/a-corn/2025h/uav-vision-control/install/setup.bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

Demo 模式任务流程：

```text
TAKEOFF (0, 0, 2.0)
  → 前20个coverage trajectory point
  → RETURN (0, 0, 2.0)
  → LAND
```

## 完整模式运行方法

完整模式用于 H 题完整测试，也是任务管理节点的默认模式。不要加载 Demo 配置，直接启动：

```bash
cd /home/a-corn/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uav_control mission_manager_node
```

默认 `demo_mode=false`，保留原有完整覆盖任务和 622 个轨迹点。其他终端及任务启动命令保持不变。

也可以显式关闭 Demo 模式：

```bash
ros2 run uav_control mission_manager_node --ros-args \
  -p demo_mode:=false
```

配置文件位置：

```text
/home/a-corn/2025h/uav-vision-control/src/uav_control/config/demo_mode.yaml
```

---

## H2025 Gazebo最终验证状态

当前系统已经完成 Gazebo + PX4 SITL 环境闭环验证。

已实现：

- ROS 2 Jazzy 通信
- PX4 Offboard 控制
- 自动预流
- 自动切换 Offboard 模式
- 自动解锁 Arm
- 自动起飞
- 2 m 高度巡查飞行
- 覆盖航线执行
- 返回起飞区域
- 自动降落
- 自动检测落地
- 自动 Disarm 锁桨
- 飞行轨迹记录

验证环境：

| 项目 | 环境 |
| --- | --- |
| 操作系统 | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| 飞控 | PX4 SITL v1.15.4 |
| 仿真 | Gazebo Harmonic |
| 通信 | Micro XRCE DDS Agent |

---

## 完整Gazebo运行流程

完整运行需要打开 7 个终端，并严格按照以下顺序启动。

### 终端1：PX4

```bash
cd ~/2025h/PX4-Autopilot
make px4_sitl gz_x500
```

### 终端2：MicroXRCE DDS

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888
```

### 终端3：位置桥

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uav_control px4_position_bridge_node
```

### 终端4：任务管理

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

完整模式：

```bash
ros2 run uav_control mission_manager_node
```

Demo 模式：

```bash
ros2 run uav_control mission_manager_node \
  --ros-args \
  --params-file src/uav_control/config/demo_mode.yaml
```

完整模式与 Demo 模式二选一启动，不要同时运行两个任务管理节点。

### 终端5：PX4控制

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uav_control mission_offboard_controller
```

### 终端6：轨迹记录

```bash
cd ~/2025h/uav-vision-control
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run uav_control flight_trajectory_logger
```

### 终端7：启动任务

```bash
source /opt/ros/jazzy/setup.bash
source ~/2025h/uav-vision-control/install/setup.bash
ros2 service call /mission/start std_srvs/srv/Trigger "{}"
```

---

## 飞行结果输出

任务完成后，系统会在工程的 `log/` 目录下自动生成飞行结果：

```text
log/
├── h2025_flight_xy_*.png
│   └── 飞行二维轨迹图
├── h2025_flight_actual_*.csv
│   └── 实际飞行位置数据
└── h2025_flight_planned_*.csv
    └── 规划航线数据
```

完整输出目录：

```text
/home/a-corn/2025h/uav-vision-control/log/
```

---

## Demo模式

Demo 模式用于快速展示完整任务闭环。

任务流程：

```text
起飞
  → 20个巡查航点
  → 返回
  → 降落
  → 自动锁桨
```

完整模式包含 622 个任务轨迹点：

```text
1个起飞点
  +
621个覆盖巡查点
```

Demo 模式适合快速演示自动起飞、巡查、返航、降落和锁桨流程；完整模式用于 H 题全部覆盖航线测试。

---

## Git版本信息

当前练习版本：

分支：

```text
h2025-practice
```

版本标签：

```text
v0.1-h2025-gazebo
v0.3-h2025-final-demo
```

该分支用于 2025 年 H 题自主巡查无人机练习，不影响主分支正式比赛工程。
