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

## 实验性红色中心辅助

`red_detector.py` 是独立的实验模块，面向原50 cm外圆、30 cm内圆和中央十字
目标中心新增的红色圆形贴纸。它在 **LAB** 颜色空间中找出形状、填充率和面积均
合理的红色区域，输出仅包含：

```text
{valid, cx, cy, area, confidence}
```

红色结果只可作为中心定位的辅助观测或未来的候选ROI提示，不能替代黑色同心圆
比例、同心约束和十字验证；因此也不生成或改变 `D_TARGET` 七字段协议。

当前正式相机为灰度模式，正式 `main.py` 未导入本模块、未改变主循环。待真实
红色贴纸到位后，先在 RGB565 实机帧上独立标定并验证，再决定是否让主流程同时
运行原几何验证和红色辅助。任何接入都必须保留原检测器作为最终验收条件。

离线查看单张测试图片中的候选红色区域（不连接 ROS 或 OpenMV）可运行：

```bash
python3 openmv_h7plus/tools/red_detector_offline.py path/to/test_image.jpg
```

该可选桌面工具使用已有的 `cv2`/`numpy` 环境，仅用于查看对应 LAB 色段的中心；
板载模块本身没有新增依赖。

详细备份、部署、回滚和验收步骤见`docs/openmv_deployment.md`。OpenMV IDE与ROS桥
不能同时占用USB CDC。未完成H7真机测试前，参数只是保守起点，不能声称达到15 Hz。
