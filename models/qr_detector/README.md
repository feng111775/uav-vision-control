# QR detector models

正式权重位于 `src/uav_vision/models/qr_hog_svm.xml`，是本仓库可直接加载
的 64×64 HOG + 线性 SVM
区域分类器。它用于无需深度学习依赖的 CPU 基线。正式部署可在隔离虚拟环境
训练 YOLO nano 并导出 ONNX；大型 `.pt`/`.onnx` 权重默认不进入 Git。

输入为灰度 64×64 patch，输出为 SVM decision score。ROS 混合后端对候选框
二次调用二维码解码，模型永远不负责识别编号。
