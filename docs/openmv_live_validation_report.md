# OpenMV H7 Plus无目标与ROS串口链验证报告

日期：2026-07-30。设备：OpenMV Cam H7 Plus、OV5640、OpenMV v5.0.0、
MicroPython v1.28.0-49。设备序列号仅经工具私下核验，本文不记录完整值。

## 已完成

- 设备枚举：固定别名`/dev/dtask_openmv`，VID/PID匹配，未操作Pixhawk。
- 文件备份：`~/openmv_backups/live_validation_20260730_133703`，10个文件，
  52 KiB，包含隐藏文件、清单和SHA256。
- REPL：cwd为`/flash`，根目录、文件大小和四文件开头均已读取。
- 单模块：`sys/os/time/pyb/csi/camera_config/protocol/detector`导入通过。
- 单帧：320×240灰度图；无目标检测返回有限、合法的七字段语义结果。
- 手动main：10秒117条无效目标心跳，无Traceback或MemoryError。
- 正式部署：四文件复制、sync、SHA256一致、安全卸载均通过。
- 自动main：Ctrl-D按v5.0设计只进入REPL；安全冷启动自动运行`main.py`通过。
- 冷启动20秒：76条`valid=0`、76条`CROSS_INVALID`，无异常。
- 优化前5分钟：1436条，平均4.786Hz，P95 301.6ms，最大358.4ms。
- 优化后5分钟：1850条，平均6.166Hz，P95 362.8ms，最大365.2ms；
  全部valid=0和CROSS_INVALID，0协议错误、0 Traceback、0 MemoryError、0重启。
- 性能探针：snapshot 21.453ms；阈值/blob区域7.562ms；圆搜索与内外圆配对
  289.449ms；无候选圆时十字阶段未执行；协议发送0.041ms。圆搜索是瓶颈。
- ROS真实串口：四个正式话题均为Float32MultiArray/String预期类型，30秒各
  226条、7.510Hz；最终样本均为无目标；未启动mission controller或legacy
  visual servo；三个PX4输入话题均不存在，项目控制输出为0。

## 诊断中见到的Traceback

正式main和稳定性测试没有Traceback。诊断过程出现：尝试无参数读取
`pyb.main()`导致TypeError；过长REPL探针超时；四条初版多行探针因格式导致
SyntaxError；ROS监测器首次把ROS array直接JSON编码导致主机TypeError。上述
均为诊断工具/命令问题，已修正，未破坏板载文件。

## 尚待完成

- 真实目标不存在：valid=1正样本、中心位置变化、角度变化、遮挡恢复、目标
  移除后的LOST时间均待真实目标制作完成后补测。
- 十字检测阶段耗时需在真实圆对候选出现后测量。
- 当前无目标输出与FPS低于15Hz，不能宣称性能达标。
- USB物理断开重连：同一桥进程实测状态序列为
  `CROSS_INVALID → DISCONNECTED → CROSS_INVALID`；固定别名消失期间节点保持
  运行，重新插入后自动打开`/dev/dtask_openmv`。受控关闭后所有视觉节点正常
  退出。
- 最终目标识别率、误检率和比赛场地光照鲁棒性尚未验证。
