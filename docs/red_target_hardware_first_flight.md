# Red target hardware first-flight checklist

This launch is separate from the PX4/Gazebo baseline. It connects:

```text
Pi front camera (actual frame size) ─┐
                                     ├─ selector ─ filter ─ normalized servo
H7 Plus down camera (320x240) ───────┘                    │
                                                         └─ PX4 Offboard
```

Each detection contains:

```text
valid,cx,cy,target_width,target_height,target_area,confidence,
image_width,image_height
```

The Pi node writes the dimensions of the frame it actually receives. The H7
serial bridge appends its fixed 320x240 geometry. Control uses:

```text
error_x = (cx - image_width / 2) / (image_width / 2)
error_y = (cy - image_height / 2) / (image_height / 2)
area_ratio = target_area / (image_width * image_height)
```

## Monitor-only startup

1. Keep propellers removed for the first data-link check.
2. Start the matching PX4 v1.17 Micro XRCE-DDS Agent.
3. Source ROS and this workspace.
4. Run:

   ```bash
   ros2 launch uav_vision red_target_hardware.launch.py
   ```

5. Confirm `/vision/front/detection` reports the Pi camera's actual width and
   height, while `/vision/down/detection` reports `320,240`.
6. Confirm `/vision/selected_camera` starts at `front`, red detections time out
   to zero velocity, and no `/fmu/in/*` Offboard output is published.

## First controlled test

Only after the monitor-only test, PX4 preflight checks, RC mode switch, kill
switch, position estimate, and manual LAND have all been checked:

```bash
ros2 launch uav_vision red_target_hardware.launch.py enable_offboard:=true
```

The pilot arms manually. The node never sends an arm command. Defaults are
0.8 m target altitude, 0.25 m/s XY limit, and 0.20 m/s Z limit. ALIGN holds
height and waits for the pilot to select LAND. Any observed transition away
from OFFBOARD latches `EXTERNAL_CONTROL`, stops Offboard heartbeat/setpoints,
and cannot reclaim OFFBOARD until the launch is restarted.
