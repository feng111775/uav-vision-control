#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/_ros_env.sh"

scenario="${1:-nominal}"
repeats="${2:-1}"
result_dir="${3:-${PWD}/test_results/sitl_hover}"
target_height_m="${4:-}"
commanded_hover_duration_s="${5:-}"
case "${scenario}" in
  low_short|nominal|high_long) ;;
  matrix)
    [[ -n "${target_height_m}" &&
       -n "${commanded_hover_duration_s}" ]] || {
      echo "Matrix runs require target height and hover duration" >&2
      exit 2
    }
    ;;
  *) echo "Unsupported scenario: ${scenario}" >&2; exit 2 ;;
esac
[[ "${repeats}" =~ ^[1-9][0-9]*$ ]] || {
  echo "Repeat count must be a positive integer" >&2
  exit 2
}
case "${scenario}" in
  low_short)
    target_height_m="${target_height_m:-0.5}"
    commanded_hover_duration_s="${commanded_hover_duration_s:-5.0}"
    ;;
  nominal)
    target_height_m="${target_height_m:-1.0}"
    commanded_hover_duration_s="${commanded_hover_duration_s:-10.0}"
    ;;
  high_long)
    target_height_m="${target_height_m:-1.5}"
    commanded_hover_duration_s="${commanded_hover_duration_s:-15.0}"
    ;;
esac
mkdir -p "${result_dir}"

safe_source /opt/ros/jazzy/setup.bash
safe_source "${PX4_ROS2_WS:-$HOME/px4_ros2_ws}/install/setup.bash"

write_failure_result() {
  local destination="$1"
  local reason="$2"
  python3 - "${destination}" "${scenario}" "${target_height_m}" \
    "${commanded_hover_duration_s}" "${reason}" <<'PY'
import json
import sys
from pathlib import Path

from uav_control.hover_matrix import failure_result

destination = Path(sys.argv[1])
result = failure_result(
    sys.argv[2], sys.argv[3] or None, sys.argv[4] or None, sys.argv[5])
destination.write_text(
    json.dumps(result, indent=2) + '\n', encoding='utf-8')
print(json.dumps(result, sort_keys=True))
PY
}

record_recovery_result() {
  local destination="$1"
  local passed="$2"
  local reason="$3"
  python3 - "${destination}" "${passed}" "${reason}" <<'PY'
import json
import sys

from uav_control.hover_matrix import record_mode_recovery

details = None
for line in reversed(sys.argv[3].splitlines()):
    try:
        candidate = json.loads(line)
    except json.JSONDecodeError:
        continue
    if isinstance(candidate, dict):
        details = candidate
        break
result = record_mode_recovery(
    sys.argv[1], sys.argv[2] == 'true', sys.argv[3], details)
print(json.dumps(result, sort_keys=True))
PY
}

fail_run() {
  local code="$1"
  local reason="$2"
  local run_number="${3:-1}"
  local destination="${result_dir}/${scenario}_run_${run_number}.json"
  printf 'Refusing run %s: %s\n' "${run_number}" "${reason}" >&2
  if [[ ! -e "${destination}" ]]; then
    write_failure_result "${destination}" "${reason}"
  else
    printf 'Failure result not overwritten: %s\n' "${destination}" >&2
  fi
  if ros2 node list 2>/dev/null |
      grep -qx '/mission_controller_node'; then
    echo "Residual mission_controller_node is present" >&2
  else
    echo "Confirmed: no residual mission_controller_node" >&2
  fi
  exit "${code}"
}

pgrep -af 'px4_sitl_default/bin/px4' | grep -q 'px4' ||
  fail_run 3 'PX4 SITL process was not found'
pgrep -af 'gz sim' >/dev/null ||
  fail_run 3 'Gazebo SITL process was not found'
agent_processes="$(ps -C MicroXRCEAgent -o args= 2>/dev/null || true)"
mapfile -t agent_lines < <(
  sed '/^[[:space:]]*$/d' <<<"${agent_processes}")
[[ "${#agent_lines[@]}" -eq 1 ]] ||
  fail_run 3 'expected exactly one MicroXRCEAgent instance'
if [[ ! "${agent_lines[0]}" =~ MicroXRCEAgent[[:space:]]+udp4 ]]; then
  fail_run 3 'only a UDP4 SITL MicroXRCEAgent is allowed'
fi
if [[ ! "${agent_lines[0]}" =~ -p[[:space:]]+8888([[:space:]]|$) ]]; then
  fail_run 3 'SITL MicroXRCEAgent must use UDP port 8888'
fi

publisher_count="$(
  ros2 topic info /fmu/out/vehicle_status_v1 2>/dev/null |
    awk '/Publisher count:/ {print $3}')" || publisher_count=''
[[ "${publisher_count}" == 1 ]] ||
  fail_run 3 'expected exactly one SITL vehicle_status publisher'

for ((run = 1; run <= repeats; run++)); do
  if ros2 node list 2>/dev/null | grep -qx '/mission_controller_node'; then
    fail_run 4 'mission_controller_node already exists' "${run}"
  fi
  result="${result_dir}/${scenario}_run_${run}.json"
  [[ ! -e "${result}" ]] ||
    fail_run 6 "result already exists: ${result}" "${run}"
  preflight_output=''
  preflight_rc=0
  preflight_output="$(ros2 run uav_control sitl_preflight_gate \
    --ros-args \
    -p timeout_seconds:=15.0 \
    -p stable_seconds:=3.0 \
    -p max_message_age_seconds:=1.0 2>&1)" || preflight_rc=$?
  printf '%s\n' "${preflight_output}"
  ((preflight_rc == 0)) ||
    fail_run 5 \
      "PX4 preflight readiness timed out; ${preflight_output}" "${run}"
  status="$(timeout 3 ros2 topic echo \
    /fmu/out/vehicle_status_v1 --once 2>/dev/null || true)"
  grep -q 'arming_state: 1' <<<"${status}" ||
    fail_run 5 'PX4 is not confirmed Disarmed' "${run}"
  grep -q 'failsafe: false' <<<"${status}" ||
    fail_run 5 'PX4 failsafe is active or unknown' "${run}"
  position="$(timeout 3 ros2 topic echo \
    /fmu/out/vehicle_local_position --once 2>/dev/null || true)"
  for field in xy_valid z_valid v_xy_valid v_z_valid; do
    grep -q "^${field}: true" <<<"${position}" ||
      fail_run 5 "local position ${field} is not true" "${run}"
  done
  launch_args=(
    scenario:="${scenario}"
    result_file:="${result}"
    timeout_seconds:=90.0
  )
  if [[ -n "${target_height_m}" ]]; then
    launch_args+=(target_height_m:="${target_height_m}")
  fi
  if [[ -n "${commanded_hover_duration_s}" ]]; then
    launch_args+=(
      commanded_hover_duration_s:="${commanded_hover_duration_s}")
  fi
  launch_rc=0
  ros2 launch uav_control sitl_hover.launch.py "${launch_args[@]}" ||
    launch_rc=$?
  result_rc=0
  if [[ ! -s "${result}" ]]; then
    echo "Run ${run} produced no result" >&2
    result_rc=6
  else
    python3 - "${result}" <<'PY' || result_rc=$?
import json
import sys

result = json.load(open(sys.argv[1], encoding='utf-8'))
print(json.dumps(result, sort_keys=True))
if not (
        result.get('passed')
        and result.get('final_state') == 'COMPLETE'
        and result.get('auto_disarmed')
        and result.get('failsafe') is False):
    raise SystemExit(1)
PY
  fi
  for _ in {1..50}; do
    ros2 node list 2>/dev/null |
      grep -qx '/mission_controller_node' || break
    sleep 0.1
  done
  if ros2 node list 2>/dev/null |
      grep -qx '/mission_controller_node'; then
    echo "Run ${run} left a residual mission_controller_node" >&2
    if [[ -s "${result}" ]]; then
      record_recovery_result \
        "${result}" false 'residual mission_controller_node'
    fi
    exit 7
  fi
  ((launch_rc == 0)) || {
    echo "Run ${run} launch exited with ${launch_rc}" >&2
    exit "${launch_rc}"
  }
  ((result_rc == 0)) || {
    echo "Run ${run} did not satisfy result checks" >&2
    exit "${result_rc}"
  }
  recovery_output=''
  recovery_rc=0
  recovery_output="$(ros2 run uav_control sitl_mode_recovery \
    --ros-args \
    -p simulation_mode:=true \
    -p timeout_seconds:=15.0 \
    -p safe_stable_seconds:=1.0 \
    -p preflight_stable_seconds:=3.0 \
    -p max_message_age_seconds:=1.0 2>&1)" || recovery_rc=$?
  printf '%s\n' "${recovery_output}"
  if ((recovery_rc != 0)); then
    record_recovery_result "${result}" false "${recovery_output}"
    echo "Run ${run} post-flight mode recovery failed" >&2
    exit 9
  fi
  record_recovery_result "${result}" true "${recovery_output}"
done
