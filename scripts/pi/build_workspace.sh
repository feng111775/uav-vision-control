#!/usr/bin/env bash
set -euo pipefail
run_tests=false
[ "${1:-}" = "--test" ] && run_tests=true
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
actual="$(git -C "$root/src/px4_msgs" rev-parse HEAD)"
[ "$actual" = "392e831c1f659429ca83902e66820d7094591410" ] || {
  echo "px4_msgs frozen commit mismatch: $actual" >&2; exit 1; }
source /opt/ros/jazzy/setup.bash
source "$root/install/setup.bash"
temp_root="$(mktemp -d /tmp/d-task-pi-build.XXXXXX)"
colcon build --build-base "$temp_root/build" --install-base "$temp_root/install" \
  --log-base "$temp_root/log" --symlink-install --packages-select uav_vision uav_control
if $run_tests; then
  colcon test --build-base "$temp_root/build" --install-base "$temp_root/install" \
    --log-base "$temp_root/log" --packages-select uav_vision uav_control
  colcon test-result --test-result-base "$temp_root/build" --verbose
fi
echo "$temp_root"
