# pi_camera_vision

Publishes `std_msgs/msg/Float32MultiArray` on
`/vision/h7/detection` in this order:

`[valid, cx, cy, width, height, area, confidence]`

Keep `h7_bridge_node` stopped while this publisher is running.

Pipeline with a desktop video:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pi_camera_vision pi_camera_pipeline.launch.py \
  source_type:=video source:=/absolute/path/test.avi
```

Pipeline with a USB camera:

```bash
ros2 launch pi_camera_vision pi_camera_pipeline.launch.py \
  source_type:=usb device:=0
```

Pipeline with Raspberry Pi Picamera2:

```bash
ros2 launch pi_camera_vision pi_camera_pipeline.launch.py \
  source_type:=picamera2
```

On Raspberry Pi, install Picamera2 from the Raspberry Pi OS packages. Its
import is optional and is never attempted for the desktop backends.

The standalone launch starts only the detector. The pipeline launch starts
exactly one detector plus the filter and visual servo, but never starts the
PX4 controller or `h7_bridge_node`:

```bash
ros2 launch pi_camera_vision pi_camera_pipeline.launch.py
```

With no launch arguments, the YAML defaults select `video` with an empty
path. The launch keeps the filter and visual-servo diagnostics running but
safely skips the camera node until `source` is supplied, rather than exiting
with a video-open exception. Launch arguments `source_type`, `source`, and
`device` override the corresponding YAML values for the camera node.

Installation orientation and detection thresholds are configured in
`config/pi_camera_vision.yaml`: `rotation` accepts 0/90/180/270 degrees,
the two flip parameters are booleans, and both red HSV hue ranges plus
saturation/value minima are configurable.
