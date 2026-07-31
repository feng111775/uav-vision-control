#!/usr/bin/env bash
set -euo pipefail

workspace="${D_TASK_WORKSPACE:-$HOME/px4_ros2_ws}"
unit_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
unit_path="$unit_dir/d-task-vision.service"
dry_run=false
if [[ "${1:-}" == "--dry-run" ]]; then
  dry_run=true
elif [[ $# -ne 0 ]]; then
  echo "usage: $0 [--dry-run]" >&2
  exit 2
fi

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy setup is missing" >&2
  exit 1
fi
if [[ ! -f "$workspace/install/setup.bash" ]]; then
  echo "workspace install/setup.bash is missing: $workspace" >&2
  exit 1
fi

content="[Unit]
Description=D-task OpenMV read-only vision chain
After=local-fs.target

[Service]
Type=simple
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && source \"$workspace/install/setup.bash\" && exec ros2 launch uav_vision h7_v2_readonly.launch.py port:=/dev/dtask_openmv baudrate:=115200 data_timeout_sec:=0.30 allow_legacy_protocol:=false'
Restart=on-failure
RestartSec=2
TimeoutStopSec=10
StandardOutput=journal
StandardError=journal
LogRateLimitIntervalSec=30s
LogRateLimitBurst=500

[Install]
WantedBy=default.target
"

if $dry_run; then
  printf '%s' "$content"
  exit 0
fi
mkdir -p "$unit_dir"
printf '%s' "$content" >"$unit_path"
systemctl --user daemon-reload
systemctl --user disable --now d-task-vision.service 2>/dev/null || true
echo "installed disabled read-only service template: $unit_path"
