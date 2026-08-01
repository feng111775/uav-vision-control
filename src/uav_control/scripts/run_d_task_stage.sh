#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${script_dir}/_ros_env.sh"

mode="${1:?usage: run_d_task_stage.sh MODE TARGET_STAGE [TIMEOUT]}"
target_stage="${2:?usage: run_d_task_stage.sh MODE TARGET_STAGE [TIMEOUT]}"
timeout_seconds="${3:-90.0}"
case "${mode}" in drop|dynamic_land) ;; *)
  echo "Mode must be drop or dynamic_land" >&2; exit 2 ;;
esac
pgrep -af 'px4_sitl_default/bin/px4' | grep -q 'px4' || {
  echo "Refusing: PX4 SITL process was not found" >&2; exit 3
}
pgrep -af 'gz sim' >/dev/null || {
  echo "Refusing: Gazebo SITL process was not found" >&2; exit 3
}
safe_source /opt/ros/jazzy/setup.bash
safe_source "${PX4_ROS2_WS:-$HOME/px4_ros2_ws}/install/setup.bash"
if ros2 node list 2>/dev/null | grep -qx '/mission_controller_node'; then
  echo "Refusing: mission_controller_node already exists" >&2
  exit 4
fi
ros2 launch uav_control sitl_d_task_stage.launch.py \
  mission_mode:="${mode}" target_stage:="${target_stage}" \
  timeout_seconds:="${timeout_seconds}"
