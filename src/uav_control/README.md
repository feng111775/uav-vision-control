# Stage 4C XTU mission framework

This package keeps the accepted PX4 v1.16 controller and adds five ROS 2
nodes under the `/uav_mission` interface namespace.

- `car_start_gateway` normalizes a simulated car event and rejects repeated
  `task_id` values.
- `car_marker_vision` publishes a timestamped circular-cross observation
  contract. It does not open a camera.
- `mission_offboard_controller` is the sole stage-4C node that owns
  `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, and
  `/fmu/in/vehicle_command`. It extends the accepted 4B controller so its
  prestream, ACK, localization, failsafe, timeout and NAV_LAND gates remain.
- `payload_release` enforces task/release idempotency. In `dry_run` it only
  logs a simulated 90-degree rotation and imports no GPIO library.
- `mission_manager` owns the high-level state machine and publishes only a
  high-level command and one-shot payload request.

The states are `INITIALIZING`, `WAIT_FOR_START`, `TAKEOFF`, `HOVER_STABLE`,
`TRANSIT_TO_INTERCEPT`, `SEARCH_CAR`, `ACQUIRE_CAR`, `FOLLOW_CAR`,
`DROP_ALIGN`, `RELEASE_PAYLOAD`, `RETURN_HOME`, `LAND`, `COMPLETE`,
`ABORT_RETURN`, and `EMERGENCY_LAND`. Every active state has a global or
state timeout. Height stability is continuous; stale vision is rejected;
short target loss holds and long loss returns to search. Any payload terminal
result returns home.

Interfaces use JSON in `std_msgs/String` because this is an existing
`ament_python` package and adding generated custom messages would require
changing it to a mixed CMake package. Every JSON event includes identifiers
and nanosecond timestamps where applicable.

| Interface | Type | Direction |
| --- | --- | --- |
| `/uav_mission/sim/car_start` | `std_msgs/String` | simulation input |
| `/uav_mission/events/start` | `std_msgs/String` | normalized event |
| `/uav_mission/sim/marker` | `std_msgs/Float32MultiArray` | simulation input |
| `/uav_mission/vision/marker` | `std_msgs/String` | marker contract |
| `/uav_mission/control/command` | `std_msgs/String` | manager to controller |
| `/uav_mission/control/status` | `std_msgs/String` | controller status |
| `/uav_mission/payload/request` | `std_msgs/String` | one-shot request |
| `/uav_mission/payload/result` | `std_msgs/String` | terminal result |
| `/uav_mission/sim/position` | `std_msgs/Float32MultiArray` | dry-run pose |
| `/uav_mission/sim/px4_status` | `std_msgs/String` | dry-run health |
| `/uav_mission/state` | `std_msgs/String` | state and reason |

All safety, timing, speed, intercept and field-frame transform parameters are
in `config/mission_stage4c.yaml`. The default launch is software-only:

```bash
ros2 launch uav_control uav_mission_stage4c_sim.launch.py
ros2 topic pub --once /uav_mission/sim/car_start std_msgs/msg/String \
  "{data: '{\"task_id\":\"sim-1\",\"valid\":true}'}"
```

Merely starting the launch leaves the manager in `WAIT_FOR_START`.
`enable_control`, `enable_auto_arm`, and real payload output are false.
The next hardware stages must replace only the gateway transport with the
ESP32 protocol, the marker simulation adapter with reviewed OpenCV input, and
the payload dry-run backend with a separately interlocked GPIO/PWM driver.
