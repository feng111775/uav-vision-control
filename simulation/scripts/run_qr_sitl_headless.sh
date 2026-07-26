#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
px4_root="${PX4_AUTOPILOT_ROOT:-"$workspace/../PX4-Autopilot"}"
result_root="${QR_SITL_RESULT_DIR:-"$workspace/test_results/qr_sitl_headless"}"
mkdir -p "$result_root"

children=()
cleanup() {
    for pid in "${children[@]}"; do
        kill -INT "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"

if ! pgrep -f 'MicroXRCEAgent udp4 -p 8888' >/dev/null; then
    MicroXRCEAgent udp4 -p 8888 >"$result_root/agent.log" 2>&1 &
    children+=("$!")
fi

# A protocol-level GCS heartbeat satisfies the normal PX4 datalink check
# without weakening any arming parameter.
/usr/bin/python3 -c '
import time
from pymavlink import mavutil
m=mavutil.mavlink_connection(
    "udpout:127.0.0.1:18570", source_system=255, source_component=190)
while True:
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE)
    time.sleep(.5)
' >"$result_root/gcs_heartbeat.log" 2>&1 &
children+=("$!")

(
    cd "$px4_root"
    HEADLESS=1 PX4_GZ_WORLD=qr_shelf_world \
        make px4_sitl gz_x500_downward_camera
) >"$result_root/px4_gazebo.log" 2>&1 &
children+=("$!")

for _ in $(seq 1 40); do
    if ros2 topic list 2>/dev/null |
            grep -q '/fmu/out/vehicle_status_v1'; then
        break
    fi
    sleep 1
done
ros2 topic list | grep -q '/fmu/out/vehicle_status_v1'

ros2 launch uav_vision qr_shelf_task.launch.py \
    mode:=sitl detector_backend:=opencv target_qr_id:="${TARGET_QR_ID:-7}" \
    use_sim_time:=true simulation_mode:=true \
    enable_offboard:="${ENABLE_OFFBOARD:-false}" \
    enable_auto_arm:="${ENABLE_AUTO_ARM:-false}" \
    >"$result_root/ros_launch.log" 2>&1 &
children+=("$!")

echo "Headless stack started; evidence: $result_root"
echo "Safety flags: offboard=${ENABLE_OFFBOARD:-false} auto_arm=${ENABLE_AUTO_ARM:-false}"
wait "${children[-1]}"
