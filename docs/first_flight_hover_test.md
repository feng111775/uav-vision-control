# 首飞悬停验证

本入口只运行 `mission_controller_node` 和不发布 PX4 输入话题的
`first_flight_supervisor_node`。流程为 WAIT_PX4 → WAIT_SAFETY → WAIT_START →
PRESTREAM(2 s) → REQUEST_OFFBOARD → 人工 ARM → TAKEOFF(0.50 m NED) →
HOVER_TEST(3 s) → LAND_H → WAIT_DISARM → COMPLETE。视觉、小车、投放、二次起飞和动态降落均关闭，自动 ARM 默认关闭。

启动后必须由用户调用 `/real_practise/start` Trigger 服务；服务只有在 PX4 状态、位置、姿态、静止和 failsafe 检查全部通过时才成功。
SITL 通过不等于真机通过；当前最高结论只能是 `READY_FOR_NO_PROP_BENCH`。
