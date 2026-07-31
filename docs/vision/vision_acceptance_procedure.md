# Vision acceptance procedure

电脑端先执行 `scripts/deploy/audit_pc_vision.sh`，再执行 `scripts/deploy/run_vision_acceptance.sh`。结果写入带时间戳目录，包含节点、话题、`/fmu/in/*` 审计和 benchmark JSON。

benchmark 对五个正式话题使用显式 `BEST_EFFORT + VOLATILE + KEEP_LAST` QoS，与高频 tracked/landing 发布端及可靠诊断发布端兼容。任何零帧、QoS 不兼容导致的无数据或指标不可计算都会返回非零并打印 `FAIL`。

断线验收使用 `scripts/deploy/test_vision_reconnect.sh`：脚本启动只读链路后等待人工拔插 OpenMV，重连成功应出现 `RECOVERED`，ClockMapper 和序号状态重新开始。

历史结果 `vision_results/reconnect_20260801_010831` 证明 USB 串口确实发生了断开与恢复，但旧脚本在用户按回车后才启动 benchmark，未形成有效的 ROS 完整重连验收结果；该目录保留用于追溯。

验收关注：每个新序号只发布一次；重复、乱序不重复发布；丢帧计数正确；超过 0.30 s 只产生一次 STALE 无效转换；恢复后下一新帧恢复有效输出。无效几何量必须为 NaN，米制字段保持 NaN。
