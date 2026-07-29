# D题相机与安装标定

任何工具都不会写入competition配置；审核报告后人工转录。没有实测输入时不得生成
占位内参、虚假焦距、触地高度或米制误差。

## 内参

使用平整棋盘格，推荐9×6内角点，覆盖画面中央、四角、不同倾角，至少采集15张。
方格边长必须用卡尺测量并以米输入：

```bash
PYTHONPATH=src/uav_vision python3 \
  src/uav_vision/tools/calibrate_camera_intrinsics.py \
  calibration/checkerboard camera_intrinsics.yaml \
  --columns 9 --rows 6 --square-size-m 0.024
```

工具至少要求8张可用图，输出`camera_matrix`、`distortion_coefficients`、重投影RMS。
应剔除严重模糊图并重新标定。只有实测证明畸变修正提高精度后才在H7启用。

## 安装方向

保持无人机不动，让打印目标分别沿机体前、后、左、右移动。CSV格式：

```text
motion,delta_x_px,delta_y_px
forward,1,-20
back,-1,21
left,-19,0
right,20,1
```

运行：

```bash
PYTHONPATH=src/uav_vision python3 \
  src/uav_vision/tools/calibrate_mount_direction.py direction.csv
```

结果只是`camera_x_sign/camera_y_sign`建议；必须人工验证“目标向右时飞机应向右”
的闭环方向，不能自动改competition YAML。

## 尺度

在多个已知相机光心高度采样50 cm外圆像素直径；每个高度至少10帧，CSV：

```text
outer_diameter_px,height_m
180,1.0
120,1.5
```

至少5个样本后运行：

```bash
PYTHONPATH=src/uav_vision python3 \
  src/uav_vision/tools/calibrate_target_scale.py scale.csv --output scale.json
```

拟合`height_m = a / outer_diameter_px + b`并输出RMSE和最大残差。正式目标物理直径
为外圆0.50 m、内圆0.30 m。未完成内参和尺度标定时禁止输出米制落点误差。

## 安装尺寸记录模板

| 项目 | 实测值 | 工具/方法 | 日期 |
|---|---:|---|---|
| 小车平台上表面离地高度 | 待测 | 卷尺/卡尺 | |
| 相机光心至无人机支脚底部 | 待测 | 卡尺 | |
| 测距模块至支脚底部 | 待测 | 卡尺 | |
| 相机roll/pitch/yaw偏角 | 待测 | 标定板/水平仪 | |
