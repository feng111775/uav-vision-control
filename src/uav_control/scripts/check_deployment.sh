#!/usr/bin/env bash
set -Eeuo pipefail

# Read-only preflight: this script never launches ROS, PX4 or hardware.
if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  echo "ERROR: ROS_DISTRO must be jazzy (got ${ROS_DISTRO:-unset})" >&2
  exit 2
fi
command -v ros2 >/dev/null || { echo "ERROR: ros2 not found" >&2; exit 3; }
uav_prefix="$(ros2 pkg prefix uav_control 2>/dev/null)" || {
  echo "ERROR: uav_control is not discoverable" >&2; exit 4; }
px4_prefix="$(ros2 pkg prefix px4_msgs 2>/dev/null)" || {
  echo "ERROR: px4_msgs is not discoverable" >&2; exit 5; }
config_dir="${uav_prefix}/share/uav_control/config"
for config in competition_drop.yaml competition_dynamic_land.yaml; do
  [[ -r "${config_dir}/${config}" ]] || {
    echo "ERROR: missing installed config ${config}" >&2; exit 6; }
done
python3 - "${config_dir}" <<'PY'
import pathlib
import sys
import yaml

config_dir = pathlib.Path(sys.argv[1])
for path in (config_dir / 'competition_drop.yaml',
             config_dir / 'competition_dynamic_land.yaml'):
    params = yaml.safe_load(path.read_text())['mission_controller_node']['ros__parameters']
    required = {
        'simulation_mode': False, 'enable_auto_arm': False,
        'enable_auto_disarm': False, 'auto_start_hover_test': False,
        'allow_sitl_heading_quality_bypass': False,
        'communication_only': False, 'enable_payload_release': False,
        'enable_visual_follow': True,
    }
    for key, expected in required.items():
        if params.get(key) != expected:
            raise SystemExit(f'{path.name}: unsafe {key}={params.get(key)!r}')
print('deployment check passed: configs, ROS distro, packages and safety gates')
PY
echo "uav_control=${uav_prefix}"
echo "px4_msgs=${px4_prefix}"
