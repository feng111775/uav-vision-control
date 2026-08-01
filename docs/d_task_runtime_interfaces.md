# D-task runtime interfaces (stage 1)

This document is generated from the control baseline `60ca82b` and the
detached `origin/integration/d-task-final` vision worktree `2c7c1f5`.  All
payloads below are ROS 2 `std_msgs` messages; there are no custom competition
messages in this stage.

## Frozen topics

| Topic | Type | Publisher(s) | Subscriber(s) | Rate | QoS | Payload/units | Validity and timing | Loss action |
|---|---|---|---|---|---|---|---|---|
| `/car/mission_start` | `std_msgs/msg/Bool` | `d_task_sitl_scenario_node` (simulation); hardware car bridge in integration branch | `mission_controller_node` | 10 Hz simulation / event driven hardware | depth 10 default | `data`: start request | `false` keeps `WAIT_START`; repeated `true` is idempotent in the controller | no start, no takeoff |
| `/car/progress` | `std_msgs/msg/UInt8` | `d_task_sitl_scenario_node` (simulation); hardware car bridge | `mission_controller_node` | 10 Hz simulation / 5 Hz simulator | depth 10 default | `0=idle, 1=A/start, 2=B, 3=C, 4=D, 5=A/returned` | enum value; stale car data timeout is 2.0 s in control config | abort/return path when required |
| `/vision/h7/detection` | `std_msgs/msg/Float32MultiArray` | `h7_bridge_node`, `fake_h7_node`, Gazebo detector | `target_filter_node` | H7 event driven; fake node configurable (10 Hz default) | depth 10 default | 7 values: `valid, center_x_px, center_y_px, outer_diameter_px, inner_diameter_px, angle_rad, confidence_percent` | finite; valid 0/1; non-negative center/diameters; valid target requires positive diameters and outer>=inner; confidence 0..100 | invalid frame rejected; filter emits invalid array |
| `/vision/h7/filtered_detection` | `std_msgs/msg/Float32MultiArray` | `target_filter_node` | `target_predictor_node` | event driven | depth 10 default | same 7-value detection schema | confidence threshold 50%; 3 consecutive valid frames to confirm; 3 misses tolerated, then invalid | predictor receives invalid/lost target |
| `/vision/target/tracked` | `std_msgs/msg/Float32MultiArray` | `target_predictor_node` | `landing_error_node`, `mission_controller_node`, dashboards/health nodes | 20 Hz timeout loop plus input events | depth 10 default | 12 values: detection 0..6, `velocity_x_px_s`, `velocity_y_px_s`, `predicted_x_px`, `predicted_y_px`, `target_age_ms` | finite; exact length 12; age non-negative; predictor loss timeout 0.3 s; input timeout 0.3 s | invalid tracked array; controller will not follow |
| `/vision/landing_error` | `std_msgs/msg/Float32MultiArray` | `landing_error_node` | `mission_controller_node`, dashboards | event driven | depth 10 default | 8 values: `valid, error_x_norm, error_y_norm, error_x_px, error_y_px, angle_rad, confidence_percent, target_age_ms`; normalized errors are `(pixel-center)/image-center` | finite; exact length 8; valid 0/1; confidence 0..100; age non-negative; tracked age >300 ms becomes invalid | controller holds/searches; no blind follow |
| `/uav/payload/release` | `std_msgs/msg/Bool` | `mission_controller_node` | payload bridge/mock | controller 20 Hz state loop, pulse on release | depth 10 default | `true` requests one release | only after mission interlocks; physical release disabled in stage-1 simulation | no release when disabled |
| `/uav/payload/release_ack` | `std_msgs/msg/Bool` | payload bridge or simulation scenario | `mission_controller_node` | event/status driven | depth 10 default | `true` acknowledges the pending release | acknowledgement is accepted only after a request | timeout returns home |
| `/uav/safety/ready` | `std_msgs/msg/Bool` | integration `safety_gate_node` | `mission_controller_node` | 10 Hz | depth 10 default | `true` only when safety inputs are continuously healthy | controller safety timeout is 1.0 s; hardware bypass is not enabled | remains in safety wait / fail-safe |

## PX4 control interface

`mission_controller_node` is the retained control publisher. It publishes
`OffboardControlMode`, `TrajectorySetpoint`, and `VehicleCommand` on
`/fmu/in/*` with BEST_EFFORT + TRANSIENT_LOCAL + KEEP_LAST depth 1. The
controller subscribes to PX4 status, local position, attitude, land detection,
and command acknowledgements using the same PX4 QoS. `sitl_mode_recovery`
also contains a guarded `/fmu/in/vehicle_command` publisher (depth 10), but it
is not included by the formal competition launch.

## Control-side validity and state transitions

The control adapter rejects wrong-length, NaN, Inf, invalid-flag, negative-age,
and out-of-range confidence values. It requires confidence >=60%, age <=250 ms,
and local receipt freshness <=0.5 s. `MissionLogic` requires a continuous
`visual_stable_seconds` window (0.4 s) before `SEARCH_CAR -> VISION_FOLLOW`.
The upstream filter independently requires three valid frames. On visual loss,
the controller leaves follow/descent through its bounded search or return path;
search speed is capped by configuration (`0.18 m/s` in the stage-4C SITL
profile) with a 15 s timeout and 3 m distance limit. A two-second-old frame is
therefore rejected by both source age and local freshness checks.

## Current stage-1 simulation status
`uav_control/launch/d_task_sitl.launch.py` starts the retained controller,
the integration `d_task_sitl_scenario_node`, and the formal filter,
predictor, landing-error, and dashboard nodes. It is a software-only entry
point and does not start PX4, Gazebo, MicroXRCEAgent, or hardware. PX4 state
progression still requires either a PX4 process or a dedicated test double;
the stage-1 tests use pure `MissionLogic` inputs for that portion and do not
claim SITL or flight validation.
