#!/usr/bin/env bash
set -euo pipefail

workspace="${D_TASK_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
result_root="${D_VISION_RESULTS:-$workspace/vision_results}/reconnect_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$result_root"
launch_pid=''; launch_pgid=''; benchmark_pid=''; benchmark_pgid=''; cleanup_done=0
cleanup_term_sent=0; cleanup_kill_sent=0; cleanup_status=PASS
original_rc=0

set +u
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
set -u
cd "$workspace"

serial_target() {
  readlink -f /dev/dtask_openmv 2>/dev/null || true
}
serial_owner_count() {
  local dev; dev="$(serial_target)"; [[ -n "$dev" && -e "$dev" ]] || { echo 0; return; }
  if command -v fuser >/dev/null 2>&1; then
    fuser -s "$dev" 2>/dev/null && fuser -u "$dev" 2>/dev/null | awk '{print NF}' || echo 0
  else
    echo 0
  fi
}
node_count() {
  ros2 node list 2>/dev/null | awk -v n="/$1" '$1==n {c++} END{print c+0}'
}
residual_processes() {
  pgrep -af '(^|/)(h7_bridge_node|vision_interface_node)( |$)|benchmark_ros_vision_chain\.py|h7_v2_readonly\.launch\.py' || true
}
print_residuals() {
  local p; residual_processes
  while read -r p; do
    [[ "$p" =~ ^([0-9]+) ]] || continue
    ps -o pid=,ppid=,pgid=,args= -p "${BASH_REMATCH[1]}" 2>/dev/null || true
  done < <(residual_processes)
}
preflight_fail() { echo "FAIL: $1" >&2; print_residuals >&2; echo "result_directory=$result_root"; exit 1; }

preflight_nodes=$(( $(node_count h7_bridge_node) + $(node_count vision_interface_node) + $(node_count benchmark_ros_vision_chain) ))
preflight_processes="$(residual_processes)"
preflight_residual_process_count="$(printf '%s\n' "$preflight_processes" | sed '/^$/d' | wc -l)"
preflight_residual_node_count="$preflight_nodes"
serial_owner_count_before="$(serial_owner_count)"
printf 'preflight_residual_process_count=%s\npreflight_residual_node_count=%s\nserial_owner_count_before=%s\n' \
  "$preflight_residual_process_count" "$preflight_residual_node_count" "$serial_owner_count_before" > "$result_root/process_lifecycle.env"
[[ -e /dev/dtask_openmv ]] || preflight_fail '/dev/dtask_openmv missing'
[[ "$(git branch --show-current)" == 'feature/openmv-v2-output-fix' ]] || preflight_fail 'wrong git branch'
(( preflight_residual_process_count == 0 )) || preflight_fail 'residual vision process detected'
(( preflight_residual_node_count == 0 )) || preflight_fail 'residual ROS node detected'
(( serial_owner_count_before == 0 )) || preflight_fail 'serial device already occupied'
git rev-parse HEAD | tee "$result_root/git_commit.txt"

write_lifecycle() {
  cat > "$result_root/process_lifecycle.json" <<EOF
{
  "preflight_residual_process_count": $preflight_residual_process_count,
  "preflight_residual_node_count": $preflight_residual_node_count,
  "serial_owner_count_before": $serial_owner_count_before,
  "launch_pid": "${launch_pid:-}", "launch_pgid": "${launch_pgid:-}",
  "cleanup_term_sent": $cleanup_term_sent, "cleanup_kill_sent": $cleanup_kill_sent,
  "launch_alive_after_cleanup": $([[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null && echo true || echo false),
  "bridge_alive_after_cleanup": $([[ -n "$(pgrep -x h7_bridge_node 2>/dev/null || true)" ]] && echo true || echo false),
  "interface_alive_after_cleanup": $([[ -n "$(pgrep -x vision_interface_node 2>/dev/null || true)" ]] && echo true || echo false),
  "benchmark_alive_after_cleanup": $([[ -n "$(pgrep -f 'benchmark_ros_vision_chain\\.py' 2>/dev/null || true)" ]] && echo true || echo false),
  "serial_owner_count_after": $(serial_owner_count),
  "cleanup_status": "$cleanup_status"
}
EOF
}
cleanup() {
  local rc=$?; (( cleanup_done )) && return "$rc"; cleanup_done=1; original_rc=$rc
  set +e
  if [[ -n "$benchmark_pgid" ]] && kill -0 -- "-$benchmark_pgid" 2>/dev/null || [[ -n "$benchmark_pid" ]] && kill -0 "$benchmark_pid" 2>/dev/null; then
    if [[ -n "$benchmark_pgid" ]] && kill -0 -- "-$benchmark_pgid" 2>/dev/null; then
      kill -TERM -- "-$benchmark_pgid" 2>/dev/null
    else
      kill -TERM "$benchmark_pid" 2>/dev/null
    fi
    cleanup_term_sent=1
    for _ in {1..20}; do
      if [[ -n "$benchmark_pgid" ]]; then kill -0 -- "-$benchmark_pgid" 2>/dev/null || break; else kill -0 "$benchmark_pid" 2>/dev/null || break; fi
      sleep 0.1
    done
    if [[ -n "$benchmark_pgid" ]] && kill -0 -- "-$benchmark_pgid" 2>/dev/null; then
      kill -KILL -- "-$benchmark_pgid" 2>/dev/null; cleanup_kill_sent=1
    fi
  fi
  if [[ -n "$launch_pgid" ]] && [[ "$launch_pgid" =~ ^[0-9]+$ ]] && kill -0 -- "-$launch_pgid" 2>/dev/null; then
    kill -TERM -- "-$launch_pgid" 2>/dev/null; cleanup_term_sent=1
    for _ in {1..20}; do kill -0 "$launch_pid" 2>/dev/null || break; sleep 0.1; done
    if kill -0 "$launch_pid" 2>/dev/null; then kill -KILL -- "-$launch_pgid" 2>/dev/null; cleanup_kill_sent=1; fi
  fi
  [[ -n "$benchmark_pid" ]] && wait "$benchmark_pid" 2>/dev/null || true
  [[ -n "$launch_pid" ]] && wait "$launch_pid" 2>/dev/null || true
  for _ in {1..20}; do
    [[ -z "$(residual_processes)" ]] && (( $(node_count h7_bridge_node) == 0 )) && (( $(node_count vision_interface_node) == 0 )) && break
    sleep 0.2
  done
  if [[ -n "$(residual_processes)" || "$(pgrep -x h7_bridge_node 2>/dev/null || true)" || "$(pgrep -x vision_interface_node 2>/dev/null || true)" || "$(pgrep -f 'benchmark_ros_vision_chain\\.py' 2>/dev/null || true)" || "$(serial_owner_count)" != 0 ]]; then cleanup_status=FAIL; fi
  write_lifecycle
  echo "cleanup_status=$cleanup_status"
  if [[ "$cleanup_status" != PASS && "$original_rc" == 0 ]]; then original_rc=1; fi
  trap - EXIT INT TERM HUP
  return "$original_rc"
}
trap cleanup EXIT INT TERM HUP

setsid ros2 launch uav_vision h7_v2_readonly.launch.py >"$result_root/launch.log" 2>&1 &
launch_pid=$!
launch_pgid="$(ps -o pgid= -p "$launch_pid" | tr -d ' ')"
printf 'launch_pid=%s\nlaunch_pgid=%s\n' "$launch_pid" "$launch_pgid" >> "$result_root/process_lifecycle.env"
sleep 3
setsid python3 scripts/benchmark/benchmark_ros_vision_chain.py --seconds "${D_RECONNECT_MAX_SECONDS:-180}" --require-reconnect --json >"$result_root/reconnect.json" &
benchmark_pid=$!
benchmark_pgid="$(ps -o pgid= -p "$benchmark_pid" | tr -d ' ')"
if wait "$benchmark_pid"; then benchmark_rc=0; else benchmark_rc=$?; fi
benchmark_pid=''
if (( benchmark_rc == 0 )); then echo 'reconnect_status=PASS'; else echo 'reconnect_status=FAIL'; fi
echo "reconnect_exit_code=$benchmark_rc"; echo "result_directory=$result_root"
exit "$benchmark_rc"
