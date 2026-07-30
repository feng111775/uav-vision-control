#!/usr/bin/env bash
set -euo pipefail

unit_dir="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
unit_path="$unit_dir/d-task-vision.service"
systemctl --user disable --now d-task-vision.service 2>/dev/null || true
if [[ -f "$unit_path" ]]; then
  rm -- "$unit_path"
fi
systemctl --user daemon-reload
echo "removed: $unit_path"
