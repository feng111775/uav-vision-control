#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /home/xixi/px4_ros2_ws_firstflight/install/setup.bash
set -u
cd /home/xixi/px4_ros2_ws_firstflight
LOG=logs/bench_validation/gating_checks.log
: > "$LOG"
scenarios=(position_stale position_invalid attitude_stale armed offboard failsafe zero_timestamp moving)
for scenario in "${scenarios[@]}"; do
  echo "=== $scenario ===" | tee -a "$LOG"
  python3 scripts/hardware/mock_first_flight_px4.py --scenario "$scenario" --duration 4.0 > /tmp/mock_px4_${scenario}.log 2>&1 &
  mock_pid=$!
  sleep 1.2
  ros2 service call /real_practise/start std_srvs/srv/Trigger '{}' | tee -a "$LOG"
  wait "$mock_pid" || true
  sleep 0.5
  echo | tee -a "$LOG"
done

echo "=== nominal_ready ===" | tee -a "$LOG"
python3 scripts/hardware/mock_first_flight_px4.py --scenario nominal --duration 6.0 > /tmp/mock_px4_nominal.log 2>&1 &
mock_pid=$!
sleep 1.5
ros2 topic echo /uav/mission/state --once | tee -a "$LOG"
ros2 topic echo /real_practise/status --once | tee -a "$LOG"
ros2 service call /real_practise/start std_srvs/srv/Trigger '{}' | tee -a "$LOG"
ros2 topic echo /uav/mission/state --once | tee -a "$LOG"
wait "$mock_pid" || true
