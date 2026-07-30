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

## Gazebo Harmonic 差速小车

模型结构为 `base_footprint -> base_link -> chassis_link`，左右驱动轮通过连续关节
连接到 `base_link`，后部球形支撑通过固定关节连接。坐标系遵循 ROS 约定：+x
指向车头、+y 指向车体左侧、+z 向上；正角速度绕 +z 逆时针。左轮位于 +y，
右轮位于 -y，两个轮轴均沿 y 轴。

| Gazebo 仿真默认参数 | 数值 |
|---|---:|
| 车身长 / 宽 / 高 | 0.40 / 0.28 / 0.10 m |
| 车轮半径 / 宽度 | 0.065 / 0.035 m |
| 轮距 | 0.32 m |
| 车身质量 | 3.0 kg |
| 单个车轮质量 | 0.25 kg |

这些尺寸和质量仅是基础运动仿真的默认值，不是最终真车参数。真车轮径、轮距、
质量、惯量和控制限幅均仍待实测。

启动无界面仿真：

```bash
source /opt/ros/jazzy/setup.bash
cd /home/xixi/px4_ros2_ws
source install/setup.bash
ros2 launch car_control car_basic_sim.launch.py headless:=true
```

打开 Gazebo GUI 时使用 `headless:=false`。可通过 `x:=`、`y:=`、`z:=` 和
`yaw:=` 设置初始位姿。另一个终端可检查：

```bash
ros2 control list_controllers
ros2 topic type /diff_drive_controller/cmd_vel
ros2 topic type /diff_drive_controller/odom
ros2 topic hz /joint_states
ros2 topic hz /diff_drive_controller/odom
```

速度命令话题 `/diff_drive_controller/cmd_vel` 使用
`geometry_msgs/msg/TwistStamped`，必须填写有效时间戳；里程计话题是
`/diff_drive_controller/odom`。基础运动与命令超时烟雾测试：

```bash
ros2 run car_control car_motion_smoke_node
```

测试会低速直行、转弯、停止并验证车轮反馈里程计和 0.5 s 命令超时。停止仿真时
在 launch 终端按 `Ctrl-C`，等待 Gazebo、桥接和 ROS 节点全部退出。

## 虚拟多路灰度阵列（阶段4A）

`virtual_gray_sensor_node` 不使用摄像头或 OpenCV。模型把配置的每路采样位置从车体
坐标（+x 向前、+y 向左，单位 m）变换到场地坐标，计算有限宽度采样区域到
2 cm 跑道中心线的覆盖率，并用连续边缘生成 `[0,1]` 灰度。默认 7 路位置为
`[-0.045,-0.030,-0.015,0,0.015,0.030,0.045] m`，路数和位置均可在
`config/virtual_gray_sensor.yaml` 修改。`black_line_is_active` 控制黑/白极性；
增益、偏置和默认关闭的固定种子噪声也可配置。

里程计以车辆初始位姿为零点，节点使用 `initial_world_x/y/yaw` 做二维刚体变换，
不能把 odom 零点当作场地左下角。输出为：

- `/car/line_sensor/values`：`Float32MultiArray`
- `/car/line_sensor/error`：`Float64`
- `/car/line_sensor/detected`：`Bool`
- `/car/line_sensor/total_activation`：`Float64`

误差、检测和激活量复用阶段2 `LineErrorEstimator`。本节点只发布传感器结果，不发布
速度或电机命令，也不控制车辆。本模型用于可重复仿真；真实灰度传感器仍需在真车上
标定采样范围、极性、阈值、安装位置及环境光影响。

## 当前完成与未实现内容

当前已完成 8 个核心模块及其 9 项测试，以及 Gazebo Harmonic 差速小车基础运动
模型及确定性虚拟灰度阵列。自主循线 ROS 节点、无线通信、MCU 真车驱动、
地面站界面和 PX4 联合仿真均不在本阶段实现范围。

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
