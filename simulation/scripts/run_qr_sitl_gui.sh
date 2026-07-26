#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
px4_root="${PX4_AUTOPILOT_ROOT:-"$HOME/PX4-Autopilot"}"
log_root="${QR_SITL_LOG_DIR:-"$workspace/test_results/qr_sitl_gui"}"
pid_file="$log_root/pids"
mkdir -p "$log_root"
: >"$pid_file"

cleanup() {
    "$workspace/simulation/scripts/stop_qr_sitl.sh" >/dev/null 2>&1 || true
}
trap cleanup ERR INT TERM

[[ -f /opt/ros/jazzy/setup.bash ]] || {
    echo "ROS 2 Jazzy not found" >&2; exit 2; }
[[ -d "$px4_root/.git" ]] || {
    echo "PX4 tree not found: $px4_root" >&2; exit 2; }
git -C "$px4_root" describe --tags --always | grep -q 'v1.17' || {
    echo "PX4 v1.17.0 is required" >&2; exit 2; }
command -v MicroXRCEAgent >/dev/null || {
    echo "MicroXRCEAgent not found" >&2; exit 2; }
[[ -f "$workspace/install/setup.bash" ]] || {
    echo "Workspace is not built; run colcon build --symlink-install" >&2
    exit 2
}

set +u
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"
set -u
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
"$workspace/simulation/scripts/install_px4_overlay.sh" "$px4_root" --apply \
    >"$log_root/overlay.log" 2>&1

# This runner owns the whole stack and first removes conflicting stale copies.
"$workspace/simulation/scripts/stop_qr_sitl.sh" --all-conflicts || true
: >"$pid_file"

start() {
    "$@" &
    local child=$!
    echo "$child" >>"$pid_file"
}

MicroXRCEAgent udp4 -p 8888 >"$log_root/agent.log" 2>&1 &
echo "$!" >>"$pid_file"
sleep 1
ss -lun | grep -q ':8888 ' || {
    echo "MicroXRCEAgent did not bind UDP 8888" >&2; exit 3; }

if /usr/bin/python3 -c 'import pymavlink' 2>/dev/null; then
    /usr/bin/python3 -c '
import time
from pymavlink import mavutil
m = mavutil.mavlink_connection(
    "udpout:127.0.0.1:18570", source_system=255, source_component=190)
while True:
    m.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
        mavutil.mavlink.MAV_STATE_ACTIVE)
    time.sleep(.5)
' >"$log_root/gcs_heartbeat.log" 2>&1 &
    echo "$!" >>"$pid_file"
fi

(
    cd "$px4_root"
    # Keep PX4's console stdin open. EOF makes its prompt spin and can grow
    # logs by hundreds of MB per minute when launched in the background.
    tail -f /dev/null | PX4_GZ_WORLD=qr_shelf_world \
        make px4_sitl gz_x500_downward_camera
) >"$log_root/px4_gazebo.log" 2>&1 &
echo "$!" >>"$pid_file"

for _ in $(seq 1 60); do
    ros2 topic list 2>/dev/null | grep -q \
        '/fmu/out/vehicle_status_v1' && break
    sleep 1
done
ros2 topic list | grep -q '/fmu/out/vehicle_status_v1' || {
    echo "PX4 ROS topics did not appear" >&2; exit 4; }

ros2 launch uav_vision qr_shelf_task.launch.py mode:=sitl \
    target_qr_id:="${TARGET_QR_ID:-10}" inventory_mode:=full \
    simulation_mode:=true enable_offboard:=true enable_auto_arm:=true \
    use_sim_time:=true >"$log_root/ros_launch.log" 2>&1 &
echo "$!" >>"$pid_file"

trap - ERR
echo "GUI QR SITL started. Logs: $log_root"
echo "Open camera view: ros2 run rqt_image_view rqt_image_view /vision/qr/debug_image"
wait "$(tail -n 1 "$pid_file")"
