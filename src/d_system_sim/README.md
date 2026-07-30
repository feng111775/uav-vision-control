# d_system_sim

## 模块职责

本包后续负责启动比赛 Gazebo 世界、加载小车模型、加载固定 PX4 v1.16 无人机，
并组织陆空联合仿真。

## 2026 年 D 题正式比赛场地（阶段4A）

场地参数来源于 2026 年 D 题图：白色区域为 4.00 m × 5.00 m，Gazebo 原点在
左下角，+x 向题图右侧、+y 向上、+z 竖直向上。黑线与边框标称宽度均为 2 cm。
闭合跑道左/右直线为 `x=1.50/3.00 m, y=2.00..3.50 m`，上下半圆中心分别为
`(2.25,3.50)`、`(2.25,2.00) m`，半径 0.75 m。

A/B/C/D 分别为 `(1.50,2.00)`、`(1.50,3.50)`、`(3.00,3.50)`、
`(3.00,2.00) m`。A 点有一条垂直于轨迹、长 0.40 m 的黑色起始线；没有添加
B/C/D 横线。H 无人机固定起降圆的中心为 `(1.00,1.00) m`、直径为 `0.50 m`，
其左边缘和下边缘距场地边界均为 `0.75 m`。

小车按“物理前端位于 A 点”放置，而不是把 `base_link` 放在 A 点。当前 URDF
车身长 0.40 m，前端参考坐标系距 `base_link` 为 0.20 m；因此默认
`base_link=(1.50,1.80)`、前端 `(1.50,2.00)`、`yaw=pi/2`，车头沿世界 +y
指向 B。灰度阵列中心与前端参考点重合，初始中间通道位于轨迹上。

SDF、材质和模型均在项目内，不使用 Fuel、网络贴图或在线下载，可断网运行。
生成器只使用 Python 标准库，输出确定一致。仿真尺寸不能替代赛区实际场地复测。

无界面启动：

```bash
ros2 launch d_system_sim d_task_car_sensor.launch.py headless:=true
```

GUI 观察使用 `headless:=false`；可用 `x:=`、`y:=`、`z:=`、`yaw:=` 覆盖初始位姿。
GUI 使用项目内 `config/d_task_gui.config` 的固定俯视相机：屏幕右侧对应世界 +x，
屏幕上方对应世界 +y，因而场地左下角也显示在屏幕左下。
场地、小车、灰度输出和A到B自主循线已经接入；尚未实现完整整圈运行。

阶段4B-1的 A 到 B 自主循线组合启动：

```bash
ros2 launch d_system_sim d_task_car_line_follow.launch.py \
  headless:=true auto_start:=true
```

该 launch 直接包含既有场地/小车/传感器 launch，再追加控制节点，不复制另一套
Gazebo启动流程。本阶段只验收到 B 并继续进入上弯道至少1秒，不代表完整整圈通过。

## 尚未实现内容

尚未创建 PX4 启动适配或陆空联合场景。

## 构建与运行

```bash
source /opt/ros/jazzy/setup.bash
cd /home/xixi/px4_ros2_ws
colcon build --symlink-install --packages-select d_system_sim
source install/setup.bash
ros2 run d_system_sim system_check_node
ros2 launch d_system_sim d_system_joint.launch.py
```

## 模块边界与安全原则

本包只负责编排，不复制 `car_control` 的小车控制代码，也不复制或修改无人机控制
代码。地面站由 `ground_station` 提供且必须只读、离线。PX4 必须固定使用
`/home/xixi/PX4-Autopilot-1.16.0` 的 v1.16.0 基线，不得使用其他 PX4 版本。
