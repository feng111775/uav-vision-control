#!/usr/bin/env bash
set -euo pipefail
workspace="${D_TASK_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
: "${AMENT_TRACE_SETUP_FILES:=}"
: "${AMENT_PYTHON_EXECUTABLE:=}"
: "${AMENT_CURRENT_PREFIX:=}"
: "${COLCON_TRACE:=}"
: "${COLCON_PYTHON_EXECUTABLE:=}"
set +u; source /opt/ros/jazzy/setup.bash; set -u
python3 - <<'PY'
import importlib.util
missing=[name for name in ('rclpy','serial') if importlib.util.find_spec(name) is None]
raise SystemExit('missing dependencies: '+', '.join(missing) if missing else 0)
PY
[[ -f "$workspace/install/setup.bash" ]] || { echo "missing workspace: $workspace/install/setup.bash" >&2; exit 1; }
[[ -e /dev/dtask_openmv ]] || echo 'warning: /dev/dtask_openmv not connected'
groups "${USER:-$(id -un)}" | grep -qw dialout || echo 'warning: user is not in dialout'
printf 'ROS_DISTRO=%s\nworkspace=%s\n' "${ROS_DISTRO:-unset}" "$workspace"
