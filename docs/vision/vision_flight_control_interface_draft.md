# Vision flight-control interface draft

这是接口草案，不修改飞控代码。视觉输出为：

- `/vision/target/tracked` (`TargetObservation`)：`frame_sequence`、检测/确认/测量有效性、像素中心、归一化像素误差、圆径、角度、置信度和时间戳。
- `/vision/landing_error` (`LandingError`)：归一化像素误差、确认有效性、序号和时间戳。
- `/vision/health` (`VisionHealth`)：相机、帧、算法、协议、帧率、新鲜度和安全门状态。
- `/vision/h7/status`：CONNECTED、RECOVERED、DISCONNECTED、STALE 等状态。

`metric_valid=false` 时不得使用 `forward_m`、`left_m`；当前两者恒为 NaN。`predicted=false`。无效目标的几何字段为 NaN。`capture_stamp_valid=false` 表示 OpenMV `pyb.millis()` 尚未建立可信单调映射。

`frame_sequence` 是 OpenMV uint32 序号；重复序号只处理一次，丢失序号计数，乱序不发布，回绕和 OpenMV 重启序号 1 会重置相关状态。超过 300 ms 视为过期并退出视觉控制。

预留任务输入 `/uav/mission/state`，建议枚举：0 `MISSION_IDLE`、1 `SEARCH`、2 `FOLLOW`、3 `DROP_ALIGN`。

飞控组未来必须只使用有效、未过期、已确认视觉帧；视觉节点不发布 PX4 输入，PX4 输入只能由 uav_control 唯一控制器发布。
