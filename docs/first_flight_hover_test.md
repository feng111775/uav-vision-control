# 首飞悬停验证

当前结论仅允许 `READY_FOR_NO_PROP_BENCH`，不允许带桨试飞。
`first_flight_real.launch.py` 只启动 `mission_controller_node` 与
`first_flight_supervisor_node`；不会启动视觉节点、自动解锁、投放、动态降落、
二次起飞，也不会执行 `uxrce_dds_client start`。

启动链必须严格为：系统上电 → 在树莓派启动 `MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 460800`
→ 等待 `/fmu` 话题稳定 → 启动首飞监督器与任务控制 → 人工调用
`/real_practise/start` → 程序请求 Offboard → 人工 ARM → 低高度短时悬停。
正常流程不得自动 ARM。

`/real_practise/start` 只有在以下条件全部满足时才会成功：
- `/fmu/out/vehicle_status_v1` 持续更新。
- `/fmu/out/vehicle_local_position` 或 `/fmu/out/vehicle_local_position_v1` 持续更新。
- `/fmu/out/vehicle_attitude` 持续更新。
- 本地位置、高度和航向控制有效，且速度接近零。
- 飞控处于未解锁、非 Offboard、非 failsafe 状态。
- DDS 消息时间戳非零且持续有效。
- `mission_controller_node` 已进入 `WAIT_START`，具备 setpoint 预发送条件。

视觉闭环阻塞项：视觉当前发布 `uav_interfaces` 自定义消息，而
`mission_controller_node` 仍消费 `std_msgs/msg/Float32MultiArray`。该问题在启用视觉闭环前必须修复；本阶段不修改视觉算法，也不把视觉节点加入首飞 launch。
