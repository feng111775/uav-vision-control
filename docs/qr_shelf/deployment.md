# 树莓派 4B / Coral 准备

当前成果是 PC/OpenCV 推理，不代表已完成真机部署。图像输入由 ROS 话题参数
隔离，换真实相机只需修改 `image_topic`。SVM 是 CPU 回退；未来默认建议
320×320 nano 模型导出 ONNX，再在树莓派测延迟。

Coral 需要全整数量化 TFLite，且算子必须受 Edge TPU 支持；先用代表数据集
INT8 量化，再运行 `edgetpu_compiler model.tflite` 并检查 mapped operations。
本机没有编译器，所以没有生成或声称存在 `.edgetpu`。H7 Plus 只承担轻量
前端，不训练深度模型。

实机入口为 `qr_shelf_hardware.launch.py` 和 `qr_shelf_hardware.yaml`。
它使用与 SITL 相同的 `QRMission`、视觉伺服和正式控制器，但固定
`use_sim_time=false`、`simulation_mode=false`、Offboard/自动解锁均关闭。
设备应使用持久 udev 名称（例如 `/dev/pixhawk`），不能依赖 tty 枚举顺序。
PX4 话题必须与仓库内 v1.17 `px4_msgs` 的 MESSAGE_VERSION 对齐。

树莓派顺序：检查供电和 USB → 启动 Agent/相机驱动 → 检查 MTF-01 光流与
测距 → 启动硬件 launch 观察模式 → 检查话题频率、CPU/内存和坐标系。
`simulation/scripts/check_rpi_runtime.sh` 给出运行时基线。

拆桨台架必须验证 FRD/NED 与 ROS FLU、相机 frame、距离单位、失图/失定位
Land、RC 接管及 Kill switch。装桨前还要完成相机标定、光流方向、测距量程、
电机序号、围栏和电池 failsafe；随后只做系留、低高度、低速度测试。这些
硬件步骤均未在本机验证。
