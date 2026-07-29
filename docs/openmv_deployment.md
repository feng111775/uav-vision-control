# OpenMV H7 Plus部署与回滚

本机2026-07-29未检测到OpenMV；`/dev/ttyACM0`是Pixhawk 6C，禁止向该设备复制文件。
以下步骤必须在OpenMV IDE确认设备型号和固件5.0.0后人工执行。

## 文件清单

复制到OpenMV板根目录：

1. `camera_config.py`
2. `detector.py`
3. `protocol.py`
4. `main.py`

`thresholds.py`已删除，正式算法不使用旧红色阈值。

## 部署

1. 拆桨，不启动PX4控制器，拔除Pixhawk USB避免选错端口。
2. 在OpenMV IDE确认H7 Plus、Firmware 5.0.0、MicroPython 1.28。
3. 先下载/另存板上现有全部`.py`为带日期备份。
4. 逐个复制上述四个文件，先在IDE运行但不保存为开机脚本。
5. 查看串口应周期出现：
   `D_TARGET,...`和`D_STATUS,TRACKING|LOST|CROSS_INVALID|DETECT_ERROR`。
6. 依次测试无目标、静止目标、移动、旋转、不均匀光照和短暂遮挡。
7. 确认无异常后才将`main.py`保存到板根目录并软重启。
8. 关闭OpenMV IDE串口，再启动ROS H7模式，避免USB CDC抢占。

回滚时删除本次四个文件并恢复备份；不要更新固件或擦除其他资产。

## 验收与调优

- 无目标持续发送合法`valid=0`心跳。
- 缺少可靠十字时D_TARGET必须无效，D_STATUS显示`CROSS_INVALID`。
- 目标移动时中心/直径变化，旋转时角度按90度周期变化。
- 丢失后不能永久保持valid=1。
- 记录`D_VISION`日志的FPS和ROS topic频率，目标尽量≥15 Hz。

需要IDE实测调整：Hough circle/line阈值、最小尺寸、内存峰值、曝光稳定时间。
未实测前不得声称H7达到15 Hz或正式识别率。
