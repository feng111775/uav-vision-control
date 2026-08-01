#!/usr/bin/env bash
set -euo pipefail

profile="${1:-real_competition}"
[[ "${profile}" == "real_competition" ]] || {
  echo "ERROR: real preflight requires profile:=real_competition" >&2
  exit 2
}

source /opt/ros/jazzy/setup.bash
source install/setup.bash

commit="$(git rev-parse HEAD)"
[[ -e /dev/dtask_openmv ]] || {
  echo "ERROR: /dev/dtask_openmv is missing" >&2
  exit 1
}

python3 - <<'PY'
from pathlib import Path
import yaml

car = yaml.safe_load(Path('src/uav_control/config/car_udp_real.yaml').read_text())
servo = yaml.safe_load(Path('src/servo_control/config/servo_real.yaml').read_text())
acceptance = yaml.safe_load(
    Path('src/uav_vision/config/openmv_downward_v1_acceptance.yaml').read_text())
if car['car_start_gateway']['ros__parameters']['udp_host'] == '127.0.0.1':
    raise SystemExit('ERROR: real UDP config is localhost')
if servo['servo_node']['ros__parameters']['dry_run'] is not False:
    raise SystemExit('ERROR: servo_real.yaml dry_run must be false')
if servo['servo_node']['ros__parameters']['gpio_pin'] != 18:
    raise SystemExit('ERROR: real servo GPIO must be 18')
vision = acceptance['vision_interface_node']['ros__parameters']
if not vision['camera_orientation_verified']:
    raise SystemExit('ERROR: camera orientation acceptance marker is false')
if not vision['coordinate_mapping_verified']:
    raise SystemExit('ERROR: coordinate mapping acceptance marker is false')
if vision['camera_mount_profile'] == 'unverified':
    raise SystemExit('ERROR: camera mount profile is unverified')
PY

ros2 topic list >/tmp/first_task_topics.txt
grep -q '^/fmu/out/vehicle_status_v1$' /tmp/first_task_topics.txt || {
  echo "ERROR: PX4 VehicleStatus topic is not present" >&2
  exit 1
}

echo "PRELIGHT_OK commit=${commit}"
