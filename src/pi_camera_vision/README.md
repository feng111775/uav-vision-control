# pi_camera_vision

Publishes `std_msgs/msg/Float32MultiArray` on
`/vision/h7/detection` in this order:

`[valid, cx, cy, width, height, area, confidence]`

Keep `h7_bridge_node` stopped while this publisher is running.

Desktop video:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pi_camera_vision pi_camera_vision_node --ros-args \
  -p source_type:=video -p source:=/absolute/path/test.avi
```

USB camera uses `-p source_type:=usb -p device:=0`. On Raspberry Pi,
install Picamera2 from the Raspberry Pi OS packages and use
`-p source_type:=picamera2`. The import is optional and is never attempted
for the desktop backends.

The standalone launch starts only the detector. The pipeline launch starts
exactly one detector plus the filter and visual servo, but never starts the
PX4 controller or `h7_bridge_node`:

```bash
ros2 launch pi_camera_vision pi_camera_pipeline.launch.py
```

Installation orientation and detection thresholds are configured in
`config/pi_camera_vision.yaml`: `rotation` accepts 0/90/180/270 degrees,
the two flip parameters are booleans, and both red HSV hue ranges plus
saturation/value minima are configurable.
