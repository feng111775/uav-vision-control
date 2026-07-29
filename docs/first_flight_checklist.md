# 首次上机悬停检查清单

> 禁止直接带桨运行。先在拆除全部桨叶的状态完成通信、模式和接管验证。

- [ ] 桨叶全部拆除，机体可靠固定，周围人员撤离。
- [ ] PX4确认为1.16.0，源码/固件hash与批准基线一致。
- [ ] `px4_msgs` 为 `release/1.16`、冻结提交 `392e831c…`。
- [ ] 七个批准的 `/fmu/` 输入输出topic名称和类型逐项确认。
- [ ] 遥控器工作正常，人工模式接管已在无桨状态验证。
- [ ] 急停/kill开关方向和动作验证，操作者全程握持遥控器。
- [ ] 地理围栏、最大高度和失联动作配置并复核。
- [ ] 电池固定、电压和健康状态正常。
- [ ] 本地定位有效且静止不漂移，航向与机头方向一致。
- [ ] H点从稳定VehicleLocalPosition记录，不假设绝对零点。
- [ ] `/uav/safety/ready`超时保护验证。
- [ ] `enable_auto_arm=false`，第一次只允许人工解锁。
- [ ] `target_altitude=1.0`，第一次只飞1米、悬停10秒。
- [ ] 第一次 `enable_visual_follow=false`。
- [ ] 第一次 `enable_payload_release=false`、不连接执行机构。
- [ ] 第一次 `enable_dynamic_landing=false`、`enable_second_takeoff=false`。
- [ ] 先以 `enable_control=false` 验证只读状态和dashboard。
- [ ] 无桨验证退出Offboard后进入EXTERNAL_CONTROL且不重新抢权。

完成本清单仍不代表允许带桨飞行；带桨试验需要独立风险评审、场地和监护批准。
