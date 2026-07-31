# 中止与人工接管

遥控器/QGC 必须可立即接管。PX4 failsafe、数据超时、异常倾角或人工退出 Offboard
会锁存异常状态并请求一次 `VEHICLE_CMD_NAV_LAND`；程序不会反复抢回 Offboard。
任务进入 LAND_H 后停止飞行设定值，落地解除武装后进入 COMPLETE，之后不再发布控制设定值。
