#!/usr/bin/env bash
set -euo pipefail
mode="${1:-observe}"
duration="${2:-0}"
include_images="${INCLUDE_RAW_IMAGES:-false}"
free_kb="$(df -Pk . | awk 'NR==2 {print $4}')"
[ "$free_kb" -gt 1048576 ] || { echo "less than 1 GiB free" >&2; exit 1; }
output="d-task-run-$(date -u +%Y%m%dT%H%M%SZ)-${mode}"
[ ! -e "$output" ] || { echo "output exists: $output" >&2; exit 1; }
topics=(
 /fmu/out/vehicle_status_v1 /fmu/out/vehicle_local_position
 /fmu/out/vehicle_attitude /fmu/out/vehicle_command_ack
 /fmu/in/offboard_control_mode /fmu/in/trajectory_setpoint /fmu/in/vehicle_command
 /vision/h7/detection /vision/target/tracked /vision/landing_error /vision/h7/status
 /car/mission_start /car/progress /car/link/status /car/link/diagnostics
 /uav/payload/release /uav/payload/release_ack /uav/payload/status
 /uav/mission/state /uav/mission/event /uav/mission/telemetry /uav/mission/path
 /system/diagnostics /system/status /uav/safety/ready /uav/safety/reason
)
[ "$include_images" = true ] && topics+=(/vision/debug/target_canvas)
command=(ros2 bag record -o "$output" "${topics[@]}")
if [ "$duration" -gt 0 ]; then
  timeout --signal=INT "${duration}s" "${command[@]}" || [ "$?" -eq 124 ]
else
  "${command[@]}"
fi
