#!/usr/bin/env bash
set -euo pipefail
ws=${1:-/home/a-corn/px4_ros2_ws}; cd "$ws"; source /opt/ros/jazzy/setup.bash; source install/setup.bash
"$ws/scripts/deploy/check_openmv_device.sh"
exec ros2 launch uav_vision vision_realtime.launch.py
