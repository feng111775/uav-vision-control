#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch d_task_bringup competition_first_task.launch.py \
  profile:=readonly_bench

