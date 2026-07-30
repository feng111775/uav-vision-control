#!/usr/bin/env bash
set -euo pipefail
ws=${1:-/home/a-corn/px4_ros2_ws}; cd "$ws"; source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select uav_interfaces uav_vision
