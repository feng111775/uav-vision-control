# car_control

## 模块职责

本包负责后续小车编码器速度计算、左右轮速度闭环、灰度循线、A/B/C/D 状态机、
一键启动、小车遥测发送及 Gazebo 小车控制封装。

## 当前阶段完成内容

已建立 `ament_cmake`、C++17 工程骨架，提供版本与健康检查库以及最小单元测试。
launch、config、URDF、worlds、firmware 和 test 边界已经预留。

## 尚未实现内容

尚未实现电机、编码器、灰度传感器、循线算法、任务状态机、遥测协议、控制节点、
Gazebo 车辆模型或真实硬件适配。

## 构建与运行

```bash
source /opt/ros/jazzy/setup.bash
cd /home/xixi/px4_ros2_ws
colcon build --symlink-install --packages-select car_control
colcon test --packages-select car_control
```

本阶段没有可运行的电机控制节点；库仅供后续模块链接和健康检查。

## 模块边界与安全原则

本包不复制无人机控制代码，不修改 `uav_control`、`uav_vision` 或 `px4_msgs`。
地面站必须保持只读，不允许借由本包形成远程控制接口。陆空联合仿真由
`d_system_sim` 编排。无人机基线必须固定为 PX4 v1.16.0，禁止使用其他 PX4
版本。
