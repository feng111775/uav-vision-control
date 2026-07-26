# 树莓派 4B / Coral 准备

当前成果是 PC/OpenCV 推理，不代表已完成真机部署。图像输入由 ROS 话题参数
隔离，换真实相机只需修改 `image_topic`。SVM 是 CPU 回退；未来默认建议
320×320 nano 模型导出 ONNX，再在树莓派测延迟。

Coral 需要全整数量化 TFLite，且算子必须受 Edge TPU 支持；先用代表数据集
INT8 量化，再运行 `edgetpu_compiler model.tflite` 并检查 mapped operations。
本机没有编译器，所以没有生成或声称存在 `.edgetpu`。H7 Plus 只承担轻量
前端，不训练深度模型。
