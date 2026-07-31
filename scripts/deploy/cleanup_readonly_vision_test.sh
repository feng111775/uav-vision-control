#!/usr/bin/env bash
set -euo pipefail

workspace="${D_TASK_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
mode="${1:---check-only}"
[[ "$mode" == --check-only || "$mode" == --cleanup ]] || { echo 'usage: cleanup_readonly_vision_test.sh [--check-only|--cleanup]' >&2; exit 2; }

residual_processes() {
  pgrep -af '(^|/)(h7_bridge_node|vision_interface_node)( |$)|benchmark_ros_vision_chain\.py|h7_v2_readonly\.launch\.py' || true
}
serial_target() { readlink -f /dev/dtask_openmv 2>/dev/null || true; }
serial_owners() {
  local dev; dev="$(serial_target)"; [[ -n "$dev" && -e "$dev" ]] || return 0
  fuser -v "$dev" 2>/dev/null || true
}
show_state() {
  echo 'readonly_vision_processes:'; residual_processes
  echo 'readonly_vision_nodes:'
  ros2 node list 2>/dev/null | grep -E '^/(h7_bridge_node|vision_interface_node|benchmark_ros_vision_chain)$' || true
  echo 'serial_owners:'; serial_owners
}
if [[ "$mode" == --check-only ]]; then
  show_state
  exit 0
fi

# Cleanup is deliberately limited to process groups recorded by this test.
shopt -s nullglob
pidfiles=("$workspace"/vision_results/reconnect_*/process_lifecycle.env)
for envfile in "${pidfiles[@]}"; do
  unset launch_pid launch_pgid
  # shellcheck disable=SC1090
  source "$envfile"
  [[ "${launch_pid:-}" =~ ^[0-9]+$ && "${launch_pgid:-}" =~ ^[0-9]+$ ]] || continue
  cmdline="$(ps -o args= -p "$launch_pid" 2>/dev/null || true)"
  [[ "$cmdline" == *'ros2 launch uav_vision h7_v2_readonly.launch.py'* || "$cmdline" == *'ros2 launch'* ]] || continue
  if kill -0 -- "-$launch_pgid" 2>/dev/null; then
    kill -TERM -- "-$launch_pgid" 2>/dev/null || true
    for _ in {1..30}; do kill -0 -- "-$launch_pgid" 2>/dev/null || break; sleep 0.1; done
    kill -0 -- "-$launch_pgid" 2>/dev/null && kill -KILL -- "-$launch_pgid" 2>/dev/null || true
  fi
done
show_state
if [[ -n "$(residual_processes)" ]]; then
  echo 'cleanup_status=FAIL'; exit 1
fi
echo 'cleanup_status=PASS'
