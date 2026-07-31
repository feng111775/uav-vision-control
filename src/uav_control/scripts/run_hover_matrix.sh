#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/_ros_env.sh"

result_dir="${1:-${PWD}/test_results/sitl_hover_matrix}"
[[ ! -e "${result_dir}" ]] || {
  echo "Refusing: matrix result directory already exists: ${result_dir}" >&2
  exit 2
}
mkdir -p "${result_dir}"
jsonl="${result_dir}/matrix_results.jsonl"
csv_summary="${result_dir}/matrix_summary.csv"
json_summary="${result_dir}/matrix_summary.json"
: >"${jsonl}"

safe_source /opt/ros/jazzy/setup.bash
safe_source "${PX4_ROS2_WS:-$HOME/px4_ros2_ws}/install/setup.bash"

summarize() {
  python3 - "${jsonl}" "${csv_summary}" "${json_summary}" <<'PY'
import sys
from uav_control.hover_matrix import (
    load_json_lines, print_table, write_summary)

results = load_json_lines(sys.argv[1])
write_summary(results, sys.argv[2], sys.argv[3])
print_table(results)
PY
}

while read -r target_height hover_duration; do
  case_name="$(
    printf 'h%s_t%s' "${target_height}" "${hover_duration}" |
      tr '.' 'p')"
  case_dir="${result_dir}/${case_name}"
  mkdir -p "${case_dir}"
  case_log="${case_dir}/runner.log"
  case_rc=0
  if "${script_dir}/run_sitl_hover.sh" \
      matrix 1 "${case_dir}" "${target_height}" "${hover_duration}" \
      2>&1 | tee "${case_log}"; then
    case_rc=0
  else
    case_rc="${PIPESTATUS[0]}"
  fi
  if ((case_rc != 0)); then
    result_file="${case_dir}/matrix_run_1.json"
    if [[ -s "${result_file}" ]]; then
      python3 - "${result_file}" >>"${jsonl}" <<'PY'
import json
import sys

print(json.dumps(json.load(open(sys.argv[1], encoding='utf-8'))))
PY
    else
      python3 - "${result_file}" "${target_height}" \
        "${hover_duration}" "${case_log}" >>"${jsonl}" <<'PY'
import json
import sys

result = {
    'target_height_m': float(sys.argv[2]),
    'commanded_hover_duration_s': float(sys.argv[3]),
    'stable_hover_duration_s': 0.0,
    'hover_stability_passed': False,
    'mission_elapsed_s': 0.0,
    'mean_hover_height_m': None,
    'max_hover_height_m': None,
    'height_error_m': None,
    'initial_disarmed': False,
    'auto_disarmed': False,
    'failsafe': None,
    'final_state': 'NOT_STARTED',
    'passed': False,
    'reason': 'runner failed before result; see ' + sys.argv[4],
}
open(sys.argv[1], 'w', encoding='utf-8').write(
    json.dumps(result, indent=2) + '\n')
print(json.dumps(result))
PY
    fi
    summarize
    echo "Matrix stopped after failed case ${case_name}" >&2
    exit "${case_rc}"
  fi
  result_file="${case_dir}/matrix_run_1.json"
  python3 - "${result_file}" >>"${jsonl}" <<'PY'
import json
import sys

print(json.dumps(json.load(open(sys.argv[1], encoding='utf-8'))))
PY
done < <(python3 - <<'PY'
from uav_control.hover_matrix import matrix_cases

for height, duration in matrix_cases():
    print(height, duration)
PY
)

summarize
[[ "$(wc -l <"${jsonl}")" -eq 9 ]] || {
  echo "Matrix did not produce exactly nine results" >&2
  exit 8
}
