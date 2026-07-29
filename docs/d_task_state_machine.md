# D题统一任务状态机

> PX4 v1.16.0 实测说明：dynamic_land 的 SITL profile 在逻辑触地后使用
> `sitl_nav_land_after_touchdown=true` 请求 PX4 原生 NAV_LAND，以适配普通
> x500 地面模型；competition profile 不启用。二次起飞先在记录的落点 x/y
> 垂直升至巡航高度，再进入 RETURN_H。

三个模式共用安全入口：`WAIT_PX4 → WAIT_SAFETY → WAIT_START → PRESTREAM →
REQUEST_OFFBOARD → ARMING/WAIT_MANUAL_ARM → TAKEOFF`。状态由新鲜PX4状态、位置、
姿态、CommandAck、视觉误差、小车进度和连续稳定时间推进，不使用阻塞sleep。

## drop

`TAKEOFF → HOVER_CONFIRM(≥3s) → SEARCH_TARGET → FOLLOW_TARGET → DROP_ALIGN →
DROP_RELEASE(等待ack) → RETURN_H → LAND_H → WAIT_DISARM → COMPLETE`。
小车通过D点前未抛投则 `ABORT_RETURN_H`。release每任务只发一次；Ack超时/拒绝进入
有限处理后的安全降落。

## dynamic_land

`TAKEOFF → HOVER_CONFIRM → SEARCH_TARGET → FOLLOW_TARGET → LANDING_ALIGN →
DESCEND_ON_CAR → TOUCHDOWN_VERIFY → DISARM_ON_CAR → DWELL_ON_CAR(≥5s) →
SECOND_PRESTREAM → SECOND_ARM → SECOND_TAKEOFF → RETURN_H → LAND_H →
WAIT_DISARM → COMPLETE`。误差过大或目标丢失时水平速度清零并停止下降；D点已通过
则中止动态下降。

## hover_test

`WAIT_PX4 → WAIT_SAFETY → WAIT_START → PRESTREAM → REQUEST_OFFBOARD →
WAIT_MANUAL_ARM/ARMING → TAKEOFF(1m) → HOVER_TEST(10s) → LAND_H →
WAIT_DISARM → COMPLETE`。正式首飞配置禁用控制、自动解锁、视觉、抛投、动态降落和
二次起飞。

任何运行态可因PX4 failsafe、数据超时、异常倾角、命令拒绝、90秒超时或进度倒退进入
`FAILSAFE_LAND`。人工/PX4退出Offboard进入 `EXTERNAL_CONTROL`，不重新抢权。
