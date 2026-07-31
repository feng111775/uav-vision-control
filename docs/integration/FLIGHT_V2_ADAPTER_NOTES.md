# Flight-side V2 adapter notes

The flight adapter consumes only the accepted structured topics:

| Topic | Type | Control use |
|---|---|---|
| `/vision/target/tracked` | `uav_interfaces/msg/TargetObservation` | target confirmation, sequence and confidence |
| `/vision/landing_error` | `uav_interfaces/msg/LandingError` | normalized `error_x_norm`/`error_y_norm` |
| `/vision/health` | `uav_interfaces/msg/VisionHealth` | camera, protocol and closed-loop readiness |

`metric_valid` is currently false in the accepted vision chain. Therefore the
adapter never reads `forward_m` or `left_m`; control uses normalized image
errors only. The source timestamp is checked independently from the local
monotonic receive timeout. Duplicate, out-of-order, stale and malformed
frames are fail-closed. A sequence restart is accepted as a new generation,
but requires fresh confirmation before control can resume.

The image convention is origin top-left, x right-positive and y
down-positive. `vision_swap_xy`, `vision_invert_x` and `vision_invert_y` are
explicit configuration gates. Their defaults are provisional and must be
validated during no-propeller direction testing; no fixed car-speed feedforward
is added. Image error, camera mounting correction and PX4/NED velocity output
remain separate layers.

`legacy_array` remains available only for compatibility tests. Competition
profiles select `v2_structured`, so the legacy callbacks are not subscribed in
the formal path. The mock V2 launch is opt-in and never publishes `/fmu/in/*`.

The accepted vision publisher currently reports `ready_for_closed_loop=false`.
The controller consequently remains fail-closed until the vision team
explicitly changes that health field after its own release criteria are met.
