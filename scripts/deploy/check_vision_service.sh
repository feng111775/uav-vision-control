#!/usr/bin/env bash
set -euo pipefail

systemctl --user status d-task-vision.service --no-pager
journalctl --user-unit d-task-vision.service --no-pager -n "${1:-100}"
