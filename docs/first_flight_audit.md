# 首飞集成审计（2026-07-31）

工作区 `/home/xixi/px4_ros2_ws_firstflight`，基线 `ee2125d`，分支
`feature/first-flight-hover-validation`。控制冻结commit
`60ca82bea21301a8e14ec49a6b10612ecf927522` 无法在本地/远端/worktree核验；当前首飞入口基于总分支现有控制实现，等待控制组正式源码复核。

`real_practise` 原本不存在，现新增监督器、两个launch、参数和测试。总分支实际入口为
`mission_controller_node`；`mission_offboard_controller`不存在。首飞launch不启动
`offboard_control.py`、`vision_offboard_controller`、视觉节点、舵机或完整比赛任务。

| 文件 | 节点 | PX4输入话题 | 首飞启动 | 冲突 |
|---|---|---|---|---|
| `uav_control/mission_controller_node.py` | `mission_controller_node` | OffboardControlMode、TrajectorySetpoint、VehicleCommand | 是（唯一） | 否 |
| `uav_control/offboard_control.py` | `offboard_control` | 同上 | 否 | 仅在误启动时冲突 |
| `uav_control/vision_offboard_controller.py` | `vision_offboard_controller` | 同上 | 否 | 仅在误启动时冲突 |
| `real_practise/first_flight_supervisor_node.py` | `first_flight_supervisor_node` | 无 | 是 | 否 |

PX4 v1.16路径已核验为 `v1.16.0` / `6ea3539157ca358c70a515878b77077af7d4611d`。
SITL能够启动并输出XRCE连接初始化，但本次未完成ROS自动任务闭环（未运行自动ARM/Trigger验收驱动，超时清理），因此不能宣称SITL通过。`rosdep`未执行：系统rosdep尚未初始化。

结论：`NOT_READY`（在完成完整v1.16 SITL闭环及真实无桨台架前，不得试飞）。
