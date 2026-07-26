# Sequential QR shelf GUI SITL acceptance

This directory contains the small, reviewable evidence from the final real
Gazebo GUI run. No QR PNG, fake detection, test `transition()`, fake camera,
or fake landing message was used.

- `acceptance.json` records the required snake order, confirmation timestamps,
  corresponding NED scan positions (the controller only authorizes
  confirmation inside the stated 0.12 m tolerance), terminal mission states,
  and PX4 landing result.
- `front_camera_real.png` is a representative frame captured directly from
  `/camera/front/image_raw` through `ros_gz_bridge`.

The final log sequence was:

```text
QR_SCAN_MOVE → QR_SCAN_HOLD → QR_SCAN_CONFIRM → QR_SCAN_NEXT
... 24 independent confirmations ...
QR_INVENTORY_COMPLETE → TARGET_ACQUIRE → TARGET_APPROACH
→ LASER_ALIGN → LASER_CONFIRM → TRANSIT_TO_LANDING
→ DOWN_ACQUIRE(front) → ALIGN(down) → LAND → DISARM
```

PX4 printed `Landing at current position`, `Landing detected`, then
`Disarmed by external command`. Its status remained `failsafe=false`.
Only `vision_offboard_controller.py` published PX4 control commands.
