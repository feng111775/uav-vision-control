# ground_station

## 模块职责

本包负责只读接收小车遥测、离线状态显示和后续 CSV 日志。它不得提供控制发送
接口，也不得加载 CDN、远程字体、远程脚本或其他外部网络资源。

## 当前阶段完成内容

已建立 `ament_python` 骨架和 `health_check_node`。节点启动时只记录一次
`READ_ONLY`/offline 状态，不创建 publisher，不发送任何 ROS 控制话题。

## 尚未实现内容

尚未实现遥测订阅、状态页面、CSV 日志和离线 Web 界面。

## 构建与运行

```bash
source /opt/ros/jazzy/setup.bash
cd /home/xixi/px4_ros2_ws
colcon build --symlink-install --packages-select ground_station
source install/setup.bash
ros2 run ground_station health_check_node
```

## 模块边界与只读原则

本包只观察 `car_control` 后续提供的遥测，不拥有电机、任务或飞行控制权。联合
仿真由 `d_system_sim` 编排；无人机功能仍归现有无人机包所有。小车地面站必须
永久保持只读和离线。无人机基线必须固定为 PX4 v1.16.0。
