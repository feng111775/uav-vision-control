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
