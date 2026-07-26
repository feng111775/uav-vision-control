# 二维码货架任务架构

目标是盘点 4×6 白色货架上的 1–24，并瞄准参数指定编号，之后复用原有
front→down 选择、下视伺服与正式 PX4 控制器。模型只定位二维码候选框；
`QRCodeDetector` 对原图和透视/放大/CLAHE/阈值变体解码，避免把 24 个编号
错误建模为视觉类别。

数据流为：

`/camera/front/image_raw` → `qr_detector_node` →
`/vision/front/detection`（原七字段兼容接口）→ camera selector →
target filter → visual servo → `/control/vision_velocity` →
唯一正式控制器 `vision_offboard_controller.py`。

附加输出：

- `/vision/qr/detection`：`[id, valid,cx,cy,w,h,area,confidence]`
- `/vision/qr/confirmed`、`/vision/qr/inventory`：JSON
- `/vision/qr/laser_aligned`：连续像素误差软件判定
- `/vision/qr/debug_image`、`/vision/qr/diagnostics`

逻辑状态集合为 WAITING、PRESTREAM、TAKEOFF、QR_SEARCH、QR_INVENTORY、
TARGET_ACQUIRE、TARGET_APPROACH、LASER_ALIGN、LASER_CONFIRM、
TRANSIT_TO_LANDING、DOWN_ACQUIRE、ALIGN、LAND、DISARM、FAILSAFE。
任何感知状态都不能直接发 PX4 命令。二维码图像默认 0.5 s 超时，盘点默认
30 s；target 模式确认目标即完成，full 要求 24 个，timeout 到时使用已有结果。
飞控层维持 20 Hz、预流 1 s、位置/状态超时及单次 Land failsafe。
