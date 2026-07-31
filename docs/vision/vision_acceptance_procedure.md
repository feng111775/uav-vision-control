# Vision acceptance procedure

电脑端先执行 `scripts/deploy/audit_pc_vision.sh`，再执行 `scripts/deploy/run_vision_acceptance.sh`。结果写入带时间戳目录，包含节点、话题、`/fmu/in/*` 审计和 benchmark JSON。

benchmark 对五个正式话题使用显式 `BEST_EFFORT + VOLATILE + KEEP_LAST` QoS，与高频 tracked/landing 发布端及可靠诊断发布端兼容。任何零帧、QoS 不兼容导致的无数据或指标不可计算都会返回非零并打印 `FAIL`。

断线验收使用 `scripts/deploy/test_vision_reconnect.sh`：脚本启动只读链路后等待人工拔插 OpenMV，重连成功应出现 `RECOVERED`，ClockMapper 和序号状态重新开始。

历史结果 `vision_results/reconnect_20260801_010831` 证明 USB 串口确实发生了断开与恢复，但旧脚本在用户按回车后才启动 benchmark，未形成有效的 ROS 完整重连验收结果；该目录保留用于追溯。

历史结果 `vision_results/reconnect_20260801_012750` 为确认的假阳性：用户未进行物理拔插，脚本却因 STALE/状态消息和过松的恢复帧条件判定 PASS。现行流程要求设备节点真实消失并持续至少 1 秒、收到 DISCONNECTED、设备以相同 USB 序列号恢复、收到 RECOVERED，随后再验证至少 60 个稳定新帧。

结果 `vision_results/reconnect_20260801_020145` 已确认稳定基线和真实物理断开；失败原因是旧版本离线资格标志只在设备重新出现时设置，导致“仍离线且已资格”条件不可满足。修复后离线计时达到 1 秒即可资格化，并独立记录设备轮询与 DISCONNECTED 时间戳；串口重开时会丢弃首个不完整行片段。

结果 `vision_results/reconnect_20260801_021551` 无效：前一次失败测试残留一组视觉节点，本轮又启动一组节点，两个桥接节点同时读取串口，导致协议字节被拆分。现行脚本在启动前拒绝残留进程、ROS 节点或串口占用，并在退出时清理和验证整个独立进程组。

结果 `vision_results/reconnect_20260801_023201` 确认稳定基线、真实设备消失、DISCONNECTED 和进程清理均正常；失败原因是旧版 `poll_device()` 的首个 `if` 在设备持续离线时遮蔽了后续 `elif`，导致离线持续时间始终为 0。现版按“当前存在/当前不存在”两个互斥主分支累计单调时钟时长。

验收关注：每个新序号只发布一次；重复、乱序不重复发布；丢帧计数正确；超过 0.30 s 只产生一次 STALE 无效转换；恢复后下一新帧恢复有效输出。无效几何量必须为 NaN，米制字段保持 NaN。
