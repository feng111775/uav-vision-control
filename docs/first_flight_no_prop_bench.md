# 无桨台架验证

## 启动顺序

1. 确认未安装螺旋桨。
2. 树莓派启动 Agent：`MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 460800`。
3. 等待 `/fmu/out/vehicle_status_v1`、本地位置、姿态话题稳定。
4. 启动 dry-run：`ros2 launch real_practise first_flight_bench.launch.py`。
5. 观察 `/uav/safety/ready`、`/uav/mission/state`、`/real_practise/status`。
6. 人工调用：`ros2 service call /real_practise/start std_srvs/srv/Trigger {}`。
7. 核对 `PRESTREAM`、`REQUEST_OFFBOARD`、`VehicleCommandAck`、人工 ARM 路径、遥控器接管。

## 必查项

- `mission_controller_node` 是唯一允许发布 `/fmu/in/*` 的节点。
- dry-run 模式只验证命令生成、ACK 处理与状态机，不真正下发 `VehicleCommand`。
- `first_flight_real.launch.py` 不启动视觉、投放、动态降落、二次起飞或其他 PX4 控制发布者。
- Offboard 请求被拒绝、ACK 超时、DDS 中断、位置失效或人工接管时，任务状态机会停止推进。
- 节点重启后不会自动恢复到飞行状态。

## 故障注记

- PX4 侧 DDS 串口为 GPS2，对应设备 `/dev/ttyS6`；该信息仅用于排错和状态核对。
- 若 `/fmu/out/vehicle_local_position_v1` 未出现，PX4 v1.16 也可能使用 `/fmu/out/vehicle_local_position`。
- 不得在正常流程中手工执行 `uxrce_dds_client start`。
