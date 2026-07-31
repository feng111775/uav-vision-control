#!/usr/bin/env bash
set -euo pipefail
workspace="${D_TASK_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"
: "${AMENT_CURRENT_PREFIX:=}"
: "${COLCON_TRACE:=}"
: "${COLCON_PYTHON_EXECUTABLE:=}"
set +u; source /opt/ros/jazzy/setup.bash; source "$workspace/install/setup.bash"; set -u
[[ -e /dev/dtask_openmv ]] || { echo 'FAIL: /dev/dtask_openmv missing' >&2; exit 1; }
cd "$workspace"
[[ "$(git branch --show-current)" == "feature/openmv-v2-output-fix" ]] || { echo 'FAIL: wrong git branch' >&2; exit 1; }
git rev-parse HEAD
exec ros2 launch uav_vision h7_v2_readonly.launch.py port:=/dev/dtask_openmv baudrate:=115200 data_timeout_sec:=0.30 allow_legacy_protocol:=false
