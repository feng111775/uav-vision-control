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

## Stage 4C-2 PX4 v1.16 SITL

The 4C-1 launch above is a software-only dry-run and keeps PX4 control
disabled. The 4C-2 launch uses real PX4 SITL DDS position, status, command ACK,
and landed messages. Only its dedicated YAML enables control and automatic
arming. Payload release remains a dry-run.

The accepted local versions were PX4 `v1.16.0` at
`6ea3539157ca358c70a515878b77077af7d4611d`, `px4_msgs` `v1.16.2` at
`392e831c1f659429ca83902e66820d7094591410`, ROS 2 Jazzy, Gazebo Sim
8.11.0, and Micro XRCE-DDS Agent library 3.0.

The following commands were executed successfully on this machine in
separate terminals:

```bash
MicroXRCEAgent udp4 -p 8888 -v 4
```

```bash
cd /home/a-corn/PX4-Autopilot-1.16.0
make px4_sitl gz_x500
```

This headless SITL instance had no QGroundControl connection and its saved
`NAV_DLL_ACT` value required one. In the PX4 SITL console, the acceptance run
used the runtime-only setting below; it must never be copied to a real
aircraft without a separate safety review:

```text
param set NAV_DLL_ACT 0
```

After the Agent reports a session and PX4 reports time synchronization:

```bash
source /opt/ros/jazzy/setup.bash
source /home/a-corn/px4_ros2_ws/install/setup.bash
ros2 launch uav_control uav_mission_stage4c_sitl.launch.py
```

The launch starts all five task nodes and waits in `WAIT_FOR_START`. It does
not publish fake PX4 status or position. A manual simulated car event can be
sent with:

```bash
ros2 topic pub --once /uav_mission/sim/car_start std_msgs/msg/String \
  "{data: '{\"task_id\":\"sitl-manual-1\",\"valid\":true}'}"
```

An aligned simulated visual observation can be streamed without opening a
camera:

```bash
ros2 topic pub --rate 10 /uav_mission/sim/marker \
  std_msgs/msg/Float32MultiArray \
  "{data: [1.0, 0.95, 0.0, 0.0]}"
```

The verified one-run recorder injects only those car and vision inputs. It
subscribes to real PX4 output and writes a JSON timeline:

```bash
ros2 run uav_control sitl_acceptance_driver --ros-args \
  -p task_id:=sitl-acceptance-1 \
  -p result_path:=/tmp/sitl-acceptance-1.json
```

Set `exercise_vision_loss:=true` on the driver to inject both a short loss and
a long loss followed by reacquisition. A dry-run payload failure can be
injected before starting a task:

```bash
ros2 param set /payload_release simulated_result FAILED
```

Acceptance requires `/uav_mission/state` to reach `COMPLETE`, accepted ARM
and NAV_LAND acknowledgements, real `vehicle_land_detected.landed`, no
failsafe, and exactly one dry-run release. `mission_offboard_controller`
remains the sole publisher of all three PX4 input topics.

ESP32 transport, OpenCV detection, camera access, physical payload mechanics,
GPIO, and PWM remain simulated or absent. Do not use the SITL YAML with a real
flight controller. Before any propeller-free hardware test, restore real
datalink/RC policies, disable automatic arming by default, add a physical
payload inhibit, and perform a new hardware-specific safety review.
