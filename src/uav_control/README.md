# Formal D-task mission_controller_node integration

The formal competition chain is `mission_controller_node` plus
`MissionLogic`/`mission_schema.py`. The `mission_manager` and `stage4c_core`
Stage 4C launch chain below is frozen and is not part of this integration.

The controller consumes `/vision/target/tracked` and
`/vision/landing_error`, both `std_msgs/msg/Float32MultiArray`. The tracked
array has at least 12 values:
`[valid,cx,cy,outer_diameter_px,inner_diameter_px,angle,confidence,velocity_x,velocity_y,predicted_center_x,predicted_center_y,target_age_ms]`.
The landing array has at least 8 values:
`[valid,error_x_norm,error_y_norm,error_x_px,error_y_px,angle,confidence,target_age_ms]`.

The controller requires both messages to be structurally valid, finite,
`valid >= 0.5`, confidence at least `vision_min_confidence`, internal age at
most `vision_max_target_age_ms`, and locally received within
`vision_receive_timeout_sec`. Thus a stale final valid frame cannot keep
control alive after the vision publisher stops.

Camera error mapping is:

```text
(x, y) = (error_x_norm, error_y_norm)
if vision_swap_xy: (x, y) = (y, x)
mapped_x = vision_x_sign * x - vision_target_x_norm
mapped_y = vision_y_sign * y - vision_target_y_norm
```

The default signs and gains are software-test defaults only. The camera
direction is not calibrated for flight. Use `vision_swap_xy`,
`vision_x_sign`, `vision_y_sign`, the target offsets, `vision_kp_x/y`,
`vision_deadband_norm`, and the two alignment tolerances only after the
five-direction ground calibration described below. Horizontal speed is capped
at `vision_max_speed_mps <= 0.18`; `vision_max_accel_mps2` limits changes.
Visual loss ramps horizontal velocity to zero, briefly holds/re-enters
`SEARCH_CAR`, and after `vision_loss_abort_sec` enters the existing
`RETURN_HOME` safety path. It never reuses an old error for control.

Payload release uses only `/servo_command` (`std_msgs/msg/String`) with the
exact command `throw`, and `/servo/result` (`std_msgs/msg/String`). The old
`/uav/payload/release` and `/uav/payload/release_ack` Bool interface is no
longer used by the formal controller. `throw` is published once after the
existing `ALIGN_FOR_DROP` gate, and only a fresh exact `SUCCESS:throw` after
that request enters `RETURN_HOME`. `FAILED:throw:*`,
`REJECTED:throw:*`, unknown results, stale results, duplicates, and timeout
never count as success; failure and timeout enter `FAILSAFE`. The default
`enable_payload_release` is `false`, so no servo command is published and no
success is fabricated.

The existing start input remains `/car/mission_start`
(`std_msgs/msg/Bool`). Repeated `true` messages do not restart an active or
completed task. For software-only input, use:

```bash
ros2 topic pub --once /car/mission_start std_msgs/msg/Bool "{data: true}"
```

The default safe configuration is `config/first_flight_hover.yaml`: control,
arming, visual follow and payload release are disabled. Software release
testing must explicitly enable `enable_payload_release` and run only the
servo node's dry-run configuration. Never use `dry_run=false` for this
software validation.

Five-direction ground calibration: with props removed and no payload, place
the target at image center, then move it only right, left, down, up, and back
to center. Record the signs of `error_x_norm` and `error_y_norm` relative to
the desired vehicle motion, then set `vision_swap_xy` and the two signs. Check
the configured center offsets with a centered target. Do not fly until the
mapping is confirmed independently.

# Stage 4C XTU mission framework

This package keeps the accepted PX4 v1.16 controller and adds five ROS 2
nodes under the `/uav_mission` interface namespace.

- `car_start_gateway` validates the transport-neutral versioned/checksummed
  start envelope, rejects stale/replayed `session_id + start_id` requests,
  and emits only a normalized mission request.
- `car_marker_vision` publishes a timestamped circular-cross observation
  contract. It does not open a camera.
- `mission_offboard_controller` is the sole stage-4C node that owns
  `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint`, and
  `/fmu/in/vehicle_command`. It extends the accepted 4B controller so its
  prestream, ACK, localization, failsafe, timeout and NAV_LAND gates remain.
- `payload_release` journals task/release idempotency. Its only constructible
  actuator is `DryRunReleaseActuator`; `DRY_RUN_CONFIRMED` always carries
  `physical_action=false` and imports no GPIO library.
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
ros2 run uav_control sitl_acceptance_driver --ros-args \
  -p task_id:=software-dry-run-1 -p result_path:=/tmp/software-dry-run-1.json
```

Merely starting the launch leaves the manager in `WAIT_FOR_START`.
`enable_control`, `enable_auto_arm`, and real payload output are false.
The next hardware stages must add a reviewed transport adapter beneath the
existing start protocol, replace marker simulation with reviewed OpenCV, and
the payload dry-run backend with a separately interlocked GPIO/PWM driver.
No GPIO/PWM backend or physical payload output exists in this package.

## Stage 4C-2 PX4 v1.16 SITL

The 4C-1 launch above is a software-only dry-run and keeps PX4 control
disabled. The 4C-2 launch uses real PX4 SITL DDS position, status, command ACK,
and landed messages. Only its dedicated YAML enables control and automatic
arming. That YAML also requires `simulation_mode=true` and
`confirm_sitl_only=true`; a contradictory combination fails during node
construction. Payload release remains a software-only dry-run.

Physically disconnect every real Pixhawk before using the SITL launch. ROS 2
topics and parameters cannot prove with complete certainty that a DDS endpoint
is a simulator. The SITL launch must never be used with a real aircraft, and a
successful SITL run is not approval for real flight.

The accepted local versions were PX4 `v1.16.0` at
`6ea3539157ca358c70a515878b77077af7d4611d`, `px4_msgs` `v1.16.2` at
`392e831c1f659429ca83902e66820d7094591410`, ROS 2 Jazzy, Gazebo Sim
8.11.0, and Micro XRCE-DDS Agent library 3.0.

The following commands were executed successfully on this machine in
separate terminals:

```bash
MicroXRCEAgent udp4 -p 8888 -v 4
```

Raspberry Pi serial deployment uses `/dev/ttyAMA0` at `460800` baud, with
the Pixhawk GPS2 port `/dev/ttyS6` also at `460800` baud.

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
not publish fake PX4 status or position. Start injection must use the formal
protocol path; the acceptance driver builds the version, timestamp, monotonic
sender counter and SHA-256 integrity field:

```bash
ros2 run uav_control sitl_acceptance_driver --ros-args \
  -p task_id:=sitl-manual-1 -p result_path:=/tmp/sitl-manual-1.json
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

## Stage 4C-3A real-Pixhawk propeller-free entry

`uav_mission_stage4c_hardware_bench.launch.py` is a fail-closed monitoring
entry for a later, separately authorized propeller-free bench:

```bash
ros2 launch uav_control uav_mission_stage4c_hardware_bench.launch.py
```

Its dedicated YAML sets `simulation_mode=false`, `enable_control=false`,
`enable_auto_arm=false`, `confirm_sitl_only=false`, `dry_run=true`, and
`physical_release_enabled=false`. It deliberately does not launch
`mission_offboard_controller`, so the launch graph contains no PX4 input
publisher. Conflicting bench settings remain rejected if the controller is
started separately. This entry has not been used to connect or control a real
Pixhawk in stage 4C-3A.

The flight computer software runs on the Raspberry Pi; it is not flashed into
the Pixhawk. Before a later hardware bench, remove all propellers physically,
verify the PX4/RC/datalink/kill-switch policy independently, and retain an
external power-removal path. Software switches are not physical safety
interlocks. Installing propellers, real flight, real payload release, servo
wiring, GPIO, and PWM are prohibited at this stage.

`takeoff_height_m` is the authoritative positive height in metres above the
recorded local-NED Home. The generated PX4 target is
`home_z - takeoff_height_m`, because NED z is positive downward.
`target_altitude` remains only as a deprecated compatibility parameter; startup
fails if it differs from `takeoff_height_m`.

PX4 command ACK handling requires a currently pending command, matching command
number, target identifiers, current task context, and arrival inside the
current retry window. Duplicate, stale and post-reset ACKs are ignored. PX4
1.16 ACKs have no caller-generated transaction identifier, so a delayed ACK for
the same command arriving inside a later retry window cannot be distinguished
with mathematical certainty. `ACCEPTED` means PX4 accepted the command; mode,
arming, position and `vehicle_land_detected` still determine action completion.

The car-start protocol uses `protocol_version`, `session_id + start_id`, a
monotonic per-session sender counter, sender timestamp, command and SHA-256
integrity field. Accepted identifiers and counters are atomically journaled;
stale, future, corrupt, wrong-version and replayed envelopes are rejected.
Reconnect duplicates are reported as
`ALREADY_PROCESSED`, and the gateway reports `NOT_READY` or `BUSY` instead of
bypassing `mission_manager`. Reserved protocol types are `CAR_START`,
`START_ACCEPTED`, `START_REJECTED`, `ABORT`, and `HEARTBEAT`; only the in-memory
or ROS simulation transport is implemented. No serial device, UDP address or
ESP32 hardware default is guessed; unsupported transports fail closed.

Payload handling remains dry-run. It atomically journals `ACCEPTED` then
`EXECUTING`; an uncertain restart or corrupt journal enters `LOCKED`, which
requires an explicit `RESET_LOCK` operator message. A software `SUCCESS` means
`DRY_RUN_CONFIRMED` with `physical_action=false`; it does not mean an object
was released. The physical actuator class is a nonconstructible placeholder.

## Calibrated follow and release closure

After the three-frame acquisition gate, finite fresh image error passes
configurable axis swap, per-axis sign, normalization scale and camera mounting
yaw. The resulting body-frame forward/left command is transformed with the
current finite PX4 NED yaw, then speed- and acceleration-limited before the
single controller publishes one mixed horizontal-velocity/NED-z setpoint.
Stage 4C SITL uses an explicit simulated camera calibration. Real camera axis
directions, scale and mounting rotation remain uncalibrated, so this is not a
real-aircraft follow authorization.

`follow_stable` requires continuous fresh/confident finite observations,
both image errors inside the alignment window, and estimated horizontal
command below `follow_stable_speed_mps` for the configured duration. Leaving
the window resets the timer. Short loss holds the 1.50 m NED height and ramps
horizontal velocity toward zero; long loss follows the existing reacquisition
or safe-return path. Release is requested once only after this stable gate,
and every dry-run terminal result exits follow control toward recorded-Home
return and PX4 landing. `ACCEPTED` LAND ACK is not landing confirmation;
`vehicle_land_detected.landed` is required before `COMPLETE`.

## Heading-locked bounded car search

After the 1.50 m relative-Home takeoff and continuous three-second hover, the
intercept phase precedes `SEARCH_CAR`. On search entry the manager locks the
finite PX4 local-NED heading and position. The configured body-forward speed is
`search_speed_mps=0.18`; local velocity is
`vx=speed*cos(search_heading)`, `vy=speed*sin(search_heading)`. A mixed
trajectory setpoint uses horizontal velocity while holding
`home_z-takeoff_height_m`, so NED z retains the accepted 1.50 m cruise height.

Horizontal search velocity is acceleration-limited at 0.10 m/s² and
magnitude-limited to 0.18 m/s. A single detection only
enters `ACQUIRE_CAR`, which continues the same search output. Three consecutive
fresh, finite observations with confidence at least 0.60 are required before
`FOLLOW_CAR`. Candidate loss returns to SEARCH without switching controllers.
Search is bounded to 15 seconds and 3.0 m from its locked origin; either bound
uses the existing `ABORT_RETURN` path. The global 90-second timeout remains.

## Stage 4C-3D adapter and restart-safety closure

The car-start gateway now has a fail-closed transport selection contract for
`simulation`, `serial`, and `udp`. Only `simulation` is implemented. Serial or
UDP selection requires an explicit real-transport enable plus all endpoint
parameters, then still refuses construction because no reviewed device adapter
exists. No device, socket, address, baud rate, or port is guessed. Simulation
and future adapters feed the same versioned, timestamped, checksummed
`StartProtocol`. A fresh heartbeat and the mission manager's PX4-aware
readiness are both required before a start is accepted. Heartbeat loss during
an active mission uses the named `start_transport_lost` safe-return path.
Consumed start identifiers and sender counters remain in the atomically
written start journal; a corrupt journal closes the gate.

The vision contract explicitly declares image width and height, whether errors
are pixels or normalized values, and `calibration_valid`. Pixel errors are
normalized before the existing configurable axis swap, signs, scale, camera
mount rotation, body-forward/left mapping, and yaw-based NED rotation.
`real_vision_enabled=false` in every supplied configuration. Real mode refuses
startup without dimensions and valid calibration, and the current package
still refuses it after validation because no camera/detector adapter has been
reviewed. SITL alone uses an explicit 640x480 normalized simulated calibration.

The payload journal writes both its compatibility state and an unambiguous
ledger state: `REQUEST_ACCEPTED`, `EXECUTION_STARTED`,
`DRY_RUN_CONFIRMED`, `FAILED_SAFE`, or `UNKNOWN_LOCKED`. An
`EXECUTION_STARTED` restart, corrupt file, or atomic-write failure restores
`UNKNOWN_LOCKED`; it never retries automatically. Duplicate release IDs return
their persisted terminal result without executing again. `PHYSICAL_CONFIRMED`
is reserved but cannot be produced: the physical actuator placeholder still
refuses construction and all supplied configurations keep
`physical_release_enabled=false`.

Each result reports `process_execution_count`, which starts at zero for each
node process, and the atomically persisted `historical_execution_count`.
Replaying a completed release after restart therefore returns the prior
terminal result with a process count of zero while preserving the lifetime
execution total.

The ROS workspace's `src/uav_control` must resolve to the Git-tracked source
at `/home/a-corn/XTU-uav-vision-control-git/src/uav_control`. This avoids
building a stale copied package. The old workspace copy and its previous local
build/install package directories were preserved outside the workspace at
`/home/a-corn/px4_ros2_ws_4c3d_backups_20260730`; they are not runtime inputs.

## Local UDP car-start gateway

The formal `/car/mission_start` producer is `car_start_gateway`; do not run a second UDP trigger node alongside it. It accepts one ASCII frame per datagram:

```text
$EVT,<run_id>,START*HH\r\n
$CAR,<run_id>,<state>,<elapsed_ms>,<progress_permille>,<line_mask>,<line_error>,<left_pwm>,<right_pwm>,<flags>*HH\r\n
```

`HH` is ASCII XOR over the payload after `$` and before `*`. No STM32 or ESP32 firmware source was found locally, so UART pins, UART instances, Wi-Fi mode, IP addresses, ports, run-id lifetime, and firmware field semantics remain unconfirmed.

The gateway validates peer address when enabled, bounds datagram processing, publishes `/car/progress`, `/car/telemetry`, and `/car/link_alive`, and never publishes PX4 commands. Wire `progress_permille` is mapped to the frozen `0..5` stage enum only through explicit `car_point_*_progress_permille` parameters; no uncalibrated B/D claim is made. START delivery uses an atomic pending/committed journal and three bounded `Bool(true)` publications. Committed or older run IDs are rejected; a single BOOT/READY frame cannot clear the journal because the current protocol has no session or boot counter.

`config/car_udp_localhost.yaml` is computer-only, using `127.0.0.1`, temporary test ports, and `/tmp` state. It does not confirm `192.168.4.1`.

## Offline Raspberry Pi deployment checklist

These commands were not executed. Replace placeholders only after the Raspberry Pi is independently powered and its IP is confirmed:

```bash
cd <LOCAL_REPO>
git status --short
git diff --check
ping <PI_IP>
ssh <PI_USER>@<PI_IP>
hostname
whoami
ip address
pwd
ls -la /home/<PI_USER>
find /home/<PI_USER> -maxdepth 3 -type d -name '*ros2*' -print
tar -czf /tmp/uav_control-backup-$(date +%Y%m%d-%H%M%S).tar.gz <PI_WORKSPACE>/src/uav_control
rsync -a --delete-delay --exclude build --exclude install --exclude log <LOCAL_REPO>/src/uav_control/ <PI_USER>@<PI_IP>:<PI_WORKSPACE>/src/uav_control/
cd <PI_WORKSPACE>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select uav_control --symlink-install
source <PI_WORKSPACE>/install/setup.bash
ros2 run uav_control car_start_gateway --ros-args -p transport:=udp -p real_transport_enabled:=true -p udp_host:=127.0.0.1 -p udp_peer_host:=127.0.0.1
ros2 topic echo /car/link_alive
ros2 topic echo /car/mission_start
```

No command changes Wi-Fi, Netplan, NetworkManager, routes, or network services. ESP32 Wi-Fi and real UDP testing are later steps.
