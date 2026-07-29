# OpenMV H7 Plus D题正式视觉

硬件/固件基线：OpenMV H7 Plus、Firmware 5.0.0、MicroPython 1.28、QVGA
320×240、垂直向下、USB CDC。

正式算法位于`detector.py`，流程为：

1. 灰度帧Otsu自适应暗区阈值；
2. 丢失时全帧、稳定后扩张ROI，连续失败后强制恢复全帧；
3. blob几何预筛和Hough内外圆候选；
4. 30/50=0.6比例与归一化同心约束；
5. 内圆中心ROI的两条近中心、近垂直主线检测；
6. 圆强度、比例、同心、十字、边界和连续性融合confidence。

十字无效时不伪造正式目标：D_TARGET发送`valid=0`，同时D_STATUS发送
`CROSS_INVALID`。固定红色阈值已从正式代码删除。

协议：

```text
D_TARGET,valid,cx,cy,outer_diameter_px,inner_diameter_px,angle_rad,confidence
D_STATUS,TRACKING|LOST|CROSS_INVALID|DETECT_ERROR
```

部署文件只有：

```text
camera_config.py
detector.py
protocol.py
main.py
```

详细备份、部署、回滚和验收步骤见`docs/openmv_deployment.md`。OpenMV IDE与ROS桥
不能同时占用USB CDC。未完成H7真机测试前，参数只是保守起点，不能声称达到15 Hz。
