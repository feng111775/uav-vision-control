# D题第三阶段A视觉代码与数据审计

日期：2026-07-29。起始提交：
`d6ddcfb49a8c8345801208f4769bc3e9d62dd57e`。

## 数据与设备

- `datasets/qr_shelf`：256张640×480 JPEG（train 160、val 48、test 48）及
  256个YOLO文本标注；内容属于QR货架数据，不是D题30/50 cm同心圆。
- 没有发现D题同心圆实拍、不同高度/角度/光照/模糊/遮挡序列或对应标注。
- `models`有3个旧产物（JSON、PNG、XML），与D题几何目标无关。
- `lsusb`只发现Pixhawk 6C和电脑外设，没有可确认的OpenMV H7 Plus。
  `/dev/ttyACM0`对应Auterion PX4 FMU，禁止误刷。

因此不能报告实拍检出率、误检率、像素误差、OpenMV FPS或H7→ROS真实频率。

## 文件处置

| 路径/功能 | 分类 | 处置与理由 |
|---|---|---|
| `openmv_h7plus/main.py` | 正式修改 | 从红色色块切换为正式DTaskDetector；异常隔离和无目标心跳 |
| `openmv_h7plus/detector.py` | 正式重写 | 自适应亮度、圆对、同心、十字、confidence、ROI恢复 |
| `camera_config.py` | 正式修改 | QVGA灰度，降低H7内存和色温依赖 |
| `protocol.py` | 正式修改 | 保持D_TARGET；增加独立D_STATUS诊断 |
| `thresholds.py` | 安全删除 | 仅被旧正式detector引用；固定红色阈值不再有任何引用 |
| `tools/d_task_reference_detector.py` | 新增 | PC/OpenCV离线参考，不是比赛实时节点 |
| `tools/evaluate_d_task_dataset.py` | 新增 | 标注/未标注回放统计 |
| `tools/annotate_d_task_dataset.py` | 新增 | 创建CSV标注清单 |
| `tools/generate_synthetic_d_target.py` | 新增 | 合成回归，不能替代实拍 |
| `tools/calibrat*.py` | 新增 | 内参、安装方向、尺度和安全YAML工具 |
| 第一阶段schema/filter/predict/error | 正式保留 | 字段与飞控接口冻结 |
| `h7_bridge_node.py`、dashboard | 最小兼容修改 | 仅增加只读D_STATUS；数组不变 |
| `gazebo_red_target_detector_node.py` | legacy保留 | 旧测试、setup和历史Gazebo launch仍引用 |
| `visual_servo_node.py` | legacy保留 | 旧测试/launch仍引用；不进入D题正式launch |
| `camera_selector_node.py`和双相机launch | legacy保留 | 旧硬件/测试仍引用；不进入D题正式launch |
| `pi_camera_vision` | 禁止修改 | 本阶段范围外 |

正式 `d_task_vision.launch.py`只启动一个H7/fake源、filter、predictor、
landing_error和dashboard，不启动legacy visual servo或PX4控制器。

## OpenMV API确认

固件目标为OpenMV 5.0.0/MicroPython 1.28。使用的`csi.CSI`、
`snapshot(time=...)`、`Image.get_histogram().get_threshold()`、`find_blobs`、
`find_circles`、`find_lines`、`pyb.USB_VCP`均依据OpenMV 5.0.0官方文档。
仍需在OpenMV IDE验证H7 Plus上circle/line阈值、结果对象字段、内存占用和实际FPS；
未连接开发板时不作硬件通过结论。
