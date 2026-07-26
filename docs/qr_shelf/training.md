# 训练与评估

当前机器 ROS Python 没有 PyTorch，因此没有伪造 YOLO 训练。本次实际训练
可直接运行的 64×64 HOG + 线性 SVM 区域分类器（855 训练 patch），它与
二维码解码器完全分离。命令：

```bash
python3 src/uav_vision/training/train_hog_svm.py --smoke
python3 src/uav_vision/training/train_hog_svm.py
python3 src/uav_vision/training/evaluate_backends.py --limit 48
```

可选 YOLO nano/ONNX 必须放在项目虚拟环境：

```bash
python3 -m venv .venv-qr
.venv-qr/bin/pip install -r src/uav_vision/training/requirements-training.txt
# ultralytics yolo detect train model=yolo11n.pt data=datasets/qr_shelf/dataset.yaml imgsz=320
.venv-qr/bin/python src/uav_vision/training/export_onnx.py runs/detect/train/weights/best.pt
```

Ultralytics 为 AGPL-3.0；分发闭源产品前必须重新评估许可证。仓库不包含其
权重。SVM 指标和 held-out 后端指标见 results，定位成功率与解码成功率分开
记录。
