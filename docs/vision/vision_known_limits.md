# Known limits

当前只支持像素误差，`metric_valid=false`，米制前向/左向字段为 NaN；没有最终安装尺寸和正式目标实物验收结论。`ready_for_closed_loop` 永远为 false。

时间戳只有在 ClockMapper 获得稳定样本后才可信；设备断线、重启、序号回绕会清空映射和确认状态。超时无效转换只产生一次，恢复需等待新的 V2 序号。

本交付不包含 PX4、uav_control、小车、舵机或 real_practise。任何 `/fmu/in/*` 发布者都属于外部系统，视觉验收要求其数量为 0。
