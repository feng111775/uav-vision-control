# 首次上机悬停检查清单

> 2026-07-31 当前阶段只允许无桨台架验证；结论必须保持 `NOT_READY`，直到人工台架验证通过。

- [ ] 已拆除全部螺旋桨，机体固定，台架区清场。
- [ ] Holybro Pixhawk 6C Mini / `PX4_FMU_V6C` / PX4 `v1.16.0` 基线已复核。
- [ ] `px4_msgs` 为 `release/1.16`，ROS 2 为 Jazzy，树莓派为 Ubuntu 24.04 aarch64。
- [ ] PX4 参数确认 `UXRCE_DDS_CFG=202`，正常代码不会重复执行 `uxrce_dds_client start`。
- [ ] 树莓派串口链路确认：Agent 仅使用 `/dev/ttyAMA0`、`460800`。
- [ ] 无线数传占用 TELEM2；GPS2 为 PX4 串口 DDS 通道。
- [ ] 启动 `MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 460800` 后，DDS connected。
- [ ] `timesync` 已收敛，或至少 PX4 DDS 时间戳持续有效且单调更新。
- [ ] `/fmu/out/vehicle_status_v1`、本地位置、姿态话题频率与超时检查通过。
- [ ] `/uav/safety/ready` 在静止且定位有效时为 true；异常时能回落为 false。
- [ ] 启动前 `mission_controller_node` 状态为 `WAIT_START`。
- [ ] 调用 `/real_practise/start` 前后，状态按 `WAIT_START → PRESTREAM → REQUEST_OFFBOARD` 演进。
- [ ] Offboard 请求有明确 `VehicleCommandAck`；若拒绝或超时，流程进入安全状态。
- [ ] 正常流程仅允许人工 ARM；不得自动发送循环 ARM。
- [ ] 已用遥控器验证人工接管；退出 Offboard 后程序不会重新抢权。
- [ ] 已测试 Agent 退出、DDS 断开、位置话题超时，流程会停止推进并交由 PX4 保护。
- [ ] 已测试节点退出后不再发布控制目标。
- [ ] 已用 `first_flight_bench.launch.py` dry-run 模式验证命令生成与 ACK 处理，不驱动电机。
- [ ] 已明确视觉接口不一致仍是阻塞项，首飞链未接入视觉。

完成本清单后仍不代表允许带桨飞行；带桨试验需独立风险评审与批准。
