#!/usr/bin/env bash
set -euo pipefail
workspace="${D_TASK_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"
: "${AMENT_CURRENT_PREFIX:=}"
: "${COLCON_TRACE:=}"
: "${COLCON_PYTHON_EXECUTABLE:=}"
set +u; source /opt/ros/jazzy/setup.bash; source "$workspace/install/setup.bash"; set -u
[[ -e /dev/dtask_openmv ]] || { echo 'FAIL[device]: /dev/dtask_openmv missing' >&2; exit 1; }
exec "$(dirname "${BASH_SOURCE[0]}")/run_vision_acceptance.sh"
