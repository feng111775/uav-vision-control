#!/usr/bin/env bash
set -euo pipefail
workspace="${D_TASK_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
result_root="${D_VISION_RESULTS:-$workspace/vision_results}/audit_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$result_root"
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"
: "${AMENT_CURRENT_PREFIX:=}"
: "${COLCON_TRACE:=}"
: "${COLCON_PYTHON_EXECUTABLE:=}"
set +u; source /opt/ros/jazzy/setup.bash; source "$workspace/install/setup.bash"; set -u
cd "$workspace"
fail() { echo "FAIL: $1" >&2; echo 'audit_status=FAIL'; echo "result_directory=$result_root"; exit 1; }
[[ "$(git branch --show-current)" == "feature/openmv-v2-output-fix" ]] || fail 'wrong git branch'
git rev-parse HEAD | tee "$result_root/git_commit.txt"
[[ -e /dev/dtask_openmv ]] || fail '/dev/dtask_openmv missing'
ros2 node list >"$result_root/nodes.txt"
ros2 topic list >"$result_root/topics.txt"
ros2 topic list | awk '/^\/fmu\/in\// {print}' >"$result_root/px4_topics.txt"
if [[ ! -s "$result_root/px4_topics.txt" ]]; then
  echo 'audit_status=PASS'
  echo "result_directory=$result_root"
else
  echo 'audit_status=FAIL'
  echo "result_directory=$result_root"
  exit 1
fi
