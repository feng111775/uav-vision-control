# OpenMV protocol V2

V1 remains accepted: `D_TARGET,valid,cx,cy,outer,inner,angle,confidence`.

V2 emitted once per actual `snapshot()` is:

`D_TARGET_V2,frame_sequence,capture_ticks_ms,processing_us,mode,valid,cx,cy,outer,inner,angle,confidence`

Each record terminates in a newline. An invalid frame has `valid=0` and zero geometry, never a recycled position. The Pi maps OpenMV monotonic ticks conservatively; until stable samples exist, `capture_stamp_valid=false` and end-to-end latency is NaN.
