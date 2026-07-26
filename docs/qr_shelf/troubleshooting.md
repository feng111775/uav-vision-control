# 故障排查

- `model unavailable`：正常回退 OpenCV；检查 `model_path` 是否为安装后可见路径。
- 解码失败：先确认二维码 quiet zone、相机分辨率和贴图材质，再降低距离；
  模糊/遮挡数据本就可能只能定位、不能安全解码。
- 无图像：用 `ros2 topic hz /camera/front/image_raw`，确认 bridge 和 QoS。
- 无控制：默认是安全行为；检查 SITL、QGC、飞检，以及两个显式 enable 参数。
- 禁止同时启动 `offboard_control.py`；只允许 `vision_offboard_controller.py`。
- overlay 拒绝：确认 PX4 是 v1.17.0；不要绕过版本保护。
