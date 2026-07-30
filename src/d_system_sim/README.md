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
B/C/D 横线。小车默认在 A 点以 `yaw=pi/2` 沿世界 +y 起步。H 参考标记尺寸
**按题图比例建立，后续按赛区实际场地复核。**

SDF、材质和模型均在项目内，不使用 Fuel、网络贴图或在线下载，可断网运行。
生成器只使用 Python 标准库，输出确定一致。仿真尺寸不能替代赛区实际场地复测。

无界面启动：

```bash
ros2 launch d_system_sim d_task_car_sensor.launch.py headless:=true
```

GUI 观察使用 `headless:=false`；可用 `x:=`、`y:=`、`z:=`、`yaw:=` 覆盖初始位姿。
本阶段只验证场地、差速小车和虚拟灰度输出，尚未实现自主循线、状态机或整圈运行。

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
