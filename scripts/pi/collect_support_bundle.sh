#!/usr/bin/env bash
set -euo pipefail
output="${1:-d-task-support-$(date -u +%Y%m%dT%H%M%SZ).tar.gz}"
temp="$(mktemp -d /tmp/d-task-support.XXXXXX)"
trap 'rm -rf -- "$temp"' EXIT
git status --short --branch >"$temp/git-status.txt"
git log -5 --oneline >"$temp/git-log.txt"
env | grep -E '^(ROS_DISTRO|RMW_IMPLEMENTATION)=' >"$temp/environment.txt" || true
timeout 5s ros2 topic list >"$temp/topic-list.txt" 2>&1 || true
timeout 5s ros2 topic echo /system/status --once >"$temp/system-status.txt" 2>&1 || true
journalctl --user -n 300 --no-pager >"$temp/user-journal.txt" 2>&1 || true
tar -czf "$output" -C "$temp" .
echo "$output"
