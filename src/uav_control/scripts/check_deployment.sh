#!/usr/bin/env bash
set -Eeuo pipefail

if [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
  echo "ERROR: ROS_DISTRO must be jazzy" >&2
  exit 2
fi
command -v ros2 >/dev/null || { echo "ERROR: ros2 not found" >&2; exit 3; }
uav_prefix="$(ros2 pkg prefix uav_control)"
interfaces_prefix="$(ros2 pkg prefix uav_interfaces)"
vision_prefix="$(ros2 pkg prefix uav_vision)"
config_dir="${uav_prefix}/share/uav_control/config"
for name in competition_drop.yaml competition_dynamic_land.yaml; do
  [[ -r "${config_dir}/${name}" ]] || {
    echo "ERROR: missing ${name}" >&2
    exit 4
  }
done
python3 - "${config_dir}" <<'PY'
import pathlib
import sys
import yaml

config_dir = pathlib.Path(sys.argv[1])
for name in ('competition_drop.yaml', 'competition_dynamic_land.yaml'):
    params = yaml.safe_load((config_dir / name).read_text())[
        'mission_controller_node']['ros__parameters']
    required = {
        'simulation_mode': False,
        'enable_auto_arm': False,
        'enable_payload_release': False,
        'auto_start_hover_test': False,
        'vision_adapter_mode': 'v2_structured',
        'align_stable_duration_sec': 0.4,
    }
    for key, expected in required.items():
        if params.get(key) != expected:
            raise SystemExit(f'{name}: unsafe {key}={params.get(key)!r}')
print('deployment check passed: jazzy, interfaces, vision and safe V2 profiles')
PY
echo "uav_control=${uav_prefix}"
echo "uav_interfaces=${interfaces_prefix}"
echo "uav_vision=${vision_prefix}"
