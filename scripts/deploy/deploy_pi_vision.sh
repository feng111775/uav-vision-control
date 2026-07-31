#!/usr/bin/env bash
set -euo pipefail
workspace="${D_TASK_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"
: "${AMENT_CURRENT_PREFIX:=}"
: "${COLCON_TRACE:=}"
: "${COLCON_PYTHON_EXECUTABLE:=}"
set +u; source /opt/ros/jazzy/setup.bash; set -u
cd "$workspace"
colcon build --symlink-install --packages-select uav_interfaces uav_vision
source install/setup.bash
"$(dirname "${BASH_SOURCE[0]}")/install_vision_service.sh" --dry-run >"${D_VISION_UNIT_OUTPUT:-/tmp/d-task-vision.service}"
echo 'Prepared read-only vision service template; service remains disabled.'
