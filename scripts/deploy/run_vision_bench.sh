#!/usr/bin/env bash
set -euo pipefail
ws=${1:-/home/a-corn/px4_ros2_ws}; cd "$ws"; source /opt/ros/jazzy/setup.bash; source install/setup.bash
exec ros2 launch uav_vision vision_bench.launch.py
