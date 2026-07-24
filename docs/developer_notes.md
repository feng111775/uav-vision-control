# 开发注意事项

## 1. 不要轻易修改的文件

### `vision_offboard_controller.py`

该文件包含：

- PX4 Offboard心跳；
- `TrajectorySetpoint`发送；
- `VehicleCommand`发送；
- FLU到NED坐标转换；
- 起飞高度闭环；
- WAITING、PRESTREAM、TAKEOFF、VISION_CONTROL和FAILSAFE状态机；
- 位置、状态和视觉超时保护；
- 仿真自动解锁限制。

修改该文件会直接改变飞行行为。没有对应单元测试、SITL回归和安全审查时，不要
修改：

- 状态转换条件；
- 解锁条件；
- PX4 topic；
- QoS；
- 坐标符号；
- 时间戳单位；
- setpoint有效字段；
- 速度限制；
- 超时和failsafe行为。

### `visual_servo_node.py`

该文件决定图像误差到机体速度的方向和大小。不要未经实机标定修改：

- `sign_x`、`sign_y`；
- `kp_x`、`kp_y`；
- 图像中心；
- 最大速度；
- 输入超时；
- `base_link`坐标约定。

参数调整优先通过YAML完成，不要把现场参数直接写死到算法中。

### `target_filter_node.py`

确认帧数、丢失帧数、置信度门限和滤波系数共同决定目标响应延迟。修改前要同时
检查误检抑制和目标丢失响应。

### `protocol.py`和`h7_bridge_node.py`

两端共同定义H7Plus串口协议。任一端改变字段数量、顺序或数值范围时，另一端和
协议测试必须同步更新。

### ROS 2包元数据

修改节点名称、console script、launch文件名或YAML节点键时，需要同步检查：

- `setup.py`
- `package.xml`
- launch文件
- YAML文件
- README和docs

## 2. 比赛任务到来后应修改的位置

当前仓库尚无独立任务规划层。不要直接把每一道比赛题目的流程继续堆入
`vision_offboard_controller.py`。

建议新增：

```text
src/uav_control/uav_control/mission_manager.py
src/uav_control/uav_control/trajectory_generator.py
```

其中：

- `mission_manager.py`负责比赛流程、阶段切换、计时、航点和任务完成条件。
- `trajectory_generator.py`负责将任务目标转换为连续的位置、速度和yaw参考。

比赛题目变化时，优先调整：

1. `mission_manager.py`中的任务阶段；
2. 任务配置YAML中的航点、时间和阈值；
3. `trajectory_generator.py`中的新轨迹类型；
4. 新传感器对应的独立感知节点；
5. launch中启用的任务节点和参数文件。

通常不应修改：

- PX4 DDS topic；
- `VehicleCommand`基础格式；
- `TrajectorySetpoint`发送层；
- 坐标转换基础函数；
- Offboard心跳频率；
- 已验证的安全门和超时逻辑。

## 3. 如何增加新的任务逻辑

### 第一步：描述任务状态

先把比赛流程写成独立状态图，例如：

```text
IDLE
  → TAKEOFF
  → SEARCH
  → APPROACH
  → HOLD
  → RETURN
  → LAND
  → COMPLETE
```

为每个状态定义：

- 进入条件；
- 输出目标；
- 完成条件；
- 超时时间；
- 失败动作；
- 人工取消动作。

### 第二步：定义稳定接口

任务层不应直接发布PX4 `/fmu/in/*` topic。应先定义任务目标或轨迹参考接口，
至少能够表达：

```text
控制模式
参考坐标系
位置x/y/z
速度x/y/z
yaw或yawspeed
最大速度
目标容差
保持时间
消息时间戳
任务或航点编号
```

长时间任务建议使用ROS 2 Action，以支持：

- Goal；
- Feedback；
- Result；
- Cancel。

简单的任务模式切换可以使用Service，连续轨迹参考可以使用Topic。

### 第三步：实现轨迹生成

轨迹生成器应把离散任务目标转换为连续参考，并独立处理：

- 位置目标；
- 速度目标；
- 航点序列；
- 到达判定；
- 速度和加速度限制；
- yaw规划；
- 悬停时间；
- 任务取消后的安全输出。

轨迹生成器不应直接发送PX4命令。

### 第四步：建立控制权仲裁

比赛系统可能同时存在：

- 任务轨迹；
- 视觉伺服；
- 人工接管；
- 自动降落；
- failsafe。

需要明确优先级，例如：

```text
FAILSAFE
  > LAND
  > MANUAL_OVERRIDE
  > VISION_CONTROL
  > MISSION_TRAJECTORY
  > HOLD
```

任何时刻只能有一个最终参考进入PX4控制发布层。

### 第五步：增加配置和启动文件

每类任务建议增加独立YAML，而不是修改通用默认值：

```text
config/mission_<name>.yaml
launch/mission_<name>.launch.py
```

参数文件应保存：

- 航点；
- 目标高度；
- 速度限制；
- 任务超时；
- 视觉阈值；
- 到达容差；
- 失败策略。

### 第六步：补充测试

至少测试：

- 正常状态序列；
- 每个阶段超时；
- 非法任务参数；
- 航点到达判定；
- 目标丢失；
- PX4状态丢失；
- 任务取消；
- 人工接管；
- failsafe优先级；
- 不会出现多个PX4控制发布者。

纯逻辑应尽量与ROS 2接口解耦，以便使用pytest快速验证。

### 第七步：逐级验证

推荐顺序：

1. 纯单元测试；
2. 不连接PX4的topic回放；
3. PX4 SITL；
4. 无桨真机通信；
5. 无桨Offboard台架；
6. 低高度、低速度受控飞行；
7. 完整比赛流程。

禁止在未经过前一级验证时直接进入下一阶段。

## 4. 开发边界

- `offboard_control`是通信测试节点，不是正式任务控制器。
- `vehicle_status_listener`是只读诊断工具。
- `fake_h7_node`、`h7_bridge_node`和Gazebo检测节点是互斥视觉数据源。
- 不要同时运行`offboard_control`和`vision_offboard_controller`。
- 不要通过在真机上设置`simulation_mode=true`绕过真机安全门。
- 新任务节点不得直接成为第二个PX4 setpoint发布者。
- 所有坐标和符号变更都必须同时检查FLU、FRD、ENU和NED约定。
- 所有实飞测试都应保留PX4 ULog和ROS bag，并提前验证人工接管。
