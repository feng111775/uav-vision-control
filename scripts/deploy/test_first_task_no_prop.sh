#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "NO_PROP_CONTROL: automatic arm and physical servo output are disabled."
ros2 launch d_task_bringup competition_first_task.launch.py \
  profile:=no_prop_control

