#!/usr/bin/env bash
set -euo pipefail

duration="${1:-180}"
output="${2:-/tmp/first_task_vision_audit_$(date +%Y%m%d_%H%M%S).json}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

[[ "${duration}" =~ ^[0-9]+$ ]] || {
  echo "duration must be an integer number of seconds" >&2
  exit 2
}
(( duration >= 180 )) || {
  echo "duration must be at least 180 seconds" >&2
  exit 2
}

echo "Running read-only V2 vision audit for ${duration}s."
echo "Metrics are written to ${output}."
echo "The result is evidence only; it cannot enable ready_for_closed_loop."
mkdir -p "$(dirname -- "${output}")"
set +e
timeout --signal=INT --kill-after=10s "${duration}s" \
  python3 scripts/benchmark/benchmark_ros_vision_chain.py \
    --seconds "${duration}" --json >"${output}"
status=$?
set -e
if [[ -s "${output}" ]]; then
  python3 - "${output}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
data = json.loads(path.read_text())
print(json.dumps({
    "acceptance_status": data.get("acceptance_status"),
    "raw_target_v2_hz": data.get("raw_target_v2_hz"),
    "tracked_publish_hz": data.get("tracked_publish_hz"),
    "landing_error_valid_hz": data.get("landing_error_valid_hz"),
    "sequence_gap_count": data.get("sequence_gap_count"),
    "duplicate_frame_sequence_count": data.get("duplicate_frame_sequence_count"),
    "out_of_order_sequence_count": data.get("out_of_order_sequence_count"),
    "capture_stamp_valid_ratio": data.get("capture_stamp_valid_ratio"),
    "measurement_age_p95_ms": data.get("measurement_age_p95_ms"),
    "measurement_age_p99_ms": data.get("measurement_age_p99_ms"),
    "measurement_age_max_ms": data.get("measurement_age_max_ms"),
    "processing_p95_ms": data.get("processing_p95_ms"),
    "px4_input_publisher_count": data.get("px4_input_publisher_count"),
    "node_instance_counts": data.get("node_instance_counts"),
}, indent=2, sort_keys=True))
PY
else
  echo "ERROR: benchmark produced no JSON report" >&2
  exit 1
fi
exit "${status}"
