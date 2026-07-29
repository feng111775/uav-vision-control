# d_system_sim

## 模块职责

本包后续负责启动比赛 Gazebo 世界、加载小车模型、加载固定 PX4 v1.16 无人机，
并组织陆空联合仿真。

## 当前阶段完成内容

已建立 `ament_python` 编排骨架、一次性 `system_check_node` 和联合 launch 框架。
节点只检查 `ROS_DISTRO`；launch 本阶段只运行该检查，不启动 Gazebo、PX4 或模型。

## 尚未实现内容

尚未创建完整比赛世界、小车模型、PX4 启动适配、桥接配置或联合场景。

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
