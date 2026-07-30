#!/usr/bin/env bash
set -euo pipefail
ws=${1:-/home/a-corn/px4_ros2_ws}; cd "$ws"
test -z "$(git status --porcelain)" || { echo 'Refusing: worktree has local changes'; exit 2; }
git fetch origin --prune; git switch integration/d-task-final; git pull --ff-only origin integration/d-task-final
