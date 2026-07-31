# Vision–Flight Joint Debug Handoff

## Baseline and branch

- Repository: `feng111775/uav-vision-control`
- Integration baseline: `origin/integration/d-task-final`
- Joint branch: `integration/vision-flight-closed-loop`
- Accepted vision branch: `feature/openmv-v2-output-fix`
- Accepted vision commit: `3e12e6a72acc20891edf5c72c9ade34478a823cd`
- Acceptance tag: `vision-v2-pc-acceptance-20260801`
- PC hardware acceptance evidence: `/home/xixi/openmv_final_d554651_20260731_220407` and the final acceptance records under `vision_results/`.

The joint branch contains the accepted vision history via a non-fast-forward merge. The formal integration branch is not modified. Future flight work should start from this branch as `feature/flight-v2-closed-loop-adapter` and merge back here only after review.

## Formal V2 interface

The read-only chain is:

`OpenMV D_TARGET_V2 → h7_bridge_node → /vision/internal/h7/raw → vision_interface_node`

It publishes only the structured interfaces:

- `/vision/target/tracked` — `uav_interfaces/msg/TargetObservation`
- `/vision/landing_error` — `uav_interfaces/msg/LandingError`
- `/vision/health` — `uav_interfaces/msg/VisionHealth`
- `/vision/h7/status` — `std_msgs/msg/String`

High-rate vision topics use BEST_EFFORT, VOLATILE, KEEP_LAST-compatible QoS. The benchmark uses the same sensor-data-compatible subscription policy. The read-only launch starts only `h7_bridge_node` and `vision_interface_node`.

No vision node publishes any `/fmu/in/*` topic. The legacy Float32MultiArray path is not the formal V2 interface and must not be used for flight integration.

## Coordinate and validity semantics

- Image origin is top-left.
- `x` increases to the right; `y` increases downward.
- `error_x_norm = (cx - 160) / 160`.
- `error_y_norm = (cy - 120) / 120`.
- `metric_valid` is currently always `false`.
- `forward_m` and `left_m` are currently `NaN` and are not valid closed-loop inputs.
- The flight adapter should initially consume only `error_x_norm` and `error_y_norm`, after checking frame validity, freshness, confirmation, timestamp validity, and `ready_for_closed_loop`.

Invalid observations use NaN geometry and must not be converted into metric commands. A timeout or disconnect must immediately stop use of visual control input.

## Flight-group constraints

The flight group must not modify the OpenMV recognition algorithm, thresholds, circle/cross validation, `uav_interfaces` formal fields, PX4, `px4_msgs`, or vehicle-control behavior in this branch. Validate camera orientation and FLU/NED sign conventions explicitly before any control tuning.

The joint flight state is currently **NOT_READY**. `ready_for_closed_loop` remains false. The visual chain does not publish PX4 input and must not be used as a substitute for the unique `uav_control` command publisher.

Joint validation sequence is mandatory: no-propeller checks first, then controlled low-altitude testing only after independent approval. Do not start formal closed-loop flight directly from this handoff.
