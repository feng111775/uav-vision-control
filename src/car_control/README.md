# car_control

## 模块职责与核心架构

本包提供与 ROS、Gazebo、PX4 和具体 MCU 无关的 C++17 小车控制核心。后续 ROS
仿真节点和 MCU 适配层都应调用同一组纯计算接口。

```text
encoder counts ─> 速度估计 ───────────────┐
                                            ├─> 左右轮 PI ─> 归一化控制量
gray values ───> 灰度误差 ─> PD ─> 速度规划┘
                         └─> 丢线恢复状态
raw button ────> 消抖 ─> start event ─> A/B/C/D 进度状态机
```

| 模块 | 输入 | 输出 |
|---|---|---|
| 编码器估计 | 计数差、dt、每圈计数、轮半径 | rad/s、m/s、valid |
| PI | 目标/实际轮速、dt、显式增益和限幅 | `[-1,1]` 归一化控制量 |
| 灰度误差 | 任意路数归一化值、位置、极性、阈值 | 误差、检测状态、激活量 |
| PD | 误差、dt、增益、滤波和限幅 | 转向修正 |
| 速度规划 | 基础速度、误差、修正、循线状态 | 左右轮目标 m/s |
| 丢线恢复 | 检测状态、最后误差、dt | TRACKING/RECOVERY/FAULT、建议转向 |
| 按键消抖 | 原始状态、dt | 单次启动事件 |
| 进度状态机 | 启动、抽象节点、距离、比赛时间 | 进度状态和计时指标 |

## 单位约定

- 时间：s；距离和轮半径：m；线速度：m/s；角速度：rad/s。
- 编码器输入为本周期 count 差，每圈计数必须由具体硬件显式提供。
- 灰度值、位置误差和控制输出为归一化无量纲量；PD 转向修正在当前规划接口中按
  m/s 左右轮差速解释。

## A/B/C/D 状态机

状态严格按 `WAIT_START -> A_TO_B -> B_TO_C -> C_TO_D -> D_TO_A ->
FINISHED` 推进，只接受 B、C、D、A 顺序。节点需要满足最小累计距离增量和重复
触发锁定时间。节点通过仅更新状态与指标，**正常比赛过程中不因节点检测停车**。
记录 A 到 B 用时并判断是否小于 15 s，记录整圈用时并判断是否小于 90 s；
边界值 15 s/90 s 不算“小于”。

## 仿真参数与真车参数

`config/car_control_core.yaml` 只保存带单位的仿真默认值。轮径、编码器每圈计数、
传感器数量、增益、速度和阈值均不是已确认的真车参数，不得直接用于真车。
核心库不解析 YAML，调用方必须验证并显式传参。

## 当前完成与未实现内容

当前已完成 8 个核心模块、输入保护、饱和/复位行为和 Ubuntu 单元测试。仍未完成
Gazebo 差速模型、ROS 控制节点、MCU/HAL 驱动、真实传感器适配和现场调参。

## 构建、测试与运行

```bash
source /opt/ros/jazzy/setup.bash
cd /home/xixi/px4_ros2_ws
colcon build --symlink-install --packages-select car_control
colcon test --packages-select car_control --return-code-on-test-failure
colcon test-result --verbose
```

本阶段核心库没有可执行控制节点。地面站必须保持离线只读；联合仿真由
`d_system_sim` 编排；无人机基线必须固定为 PX4 v1.16.0。
