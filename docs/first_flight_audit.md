# 首飞集成审计（2026-07-31）

工作区为 `/home/xixi/px4_ros2_ws_firstflight`，当前分支为
`feature/first-flight-hover-validation`。当前 `/home/xixi/px4_ros2_ws` 主 worktree
停留在 `feature/vision-red-assist`，未被改动。

## 审计结论

- `integration/d-task-final` 当前不具备 `real_practise` 首飞入口，不能直接用于真实首飞。
- `feature/first-flight-hover-validation` 已包含 `real_practise`、`first_flight_real.launch.py`
  与首飞监督器，适合做无桨台架准备。
- `first_flight_real.launch.py` 仅启动 `mission_controller_node` 与
  `first_flight_supervisor_node`；没有视觉、投放、动态降落、二次起飞、自动 ARM、
  Agent 启动或 `uxrce_dds_client start`。
- `mission_controller_node` 仍订阅 `std_msgs/msg/Float32MultiArray` 视觉接口；视觉链同时存在
  `uav_interfaces` 自定义消息发布者。该不一致必须在视觉闭环启用前修复。
- 新增 `dry_run_px4_commands`，用于无桨阶段验证命令生成、ACK 处理和状态机，不绕过 PX4 安全检查。

## 当前总判断

结论保持 `NOT_READY`：代码已准备无桨台架验证，但在人工完成台架清单前，绝不允许带桨试飞。
