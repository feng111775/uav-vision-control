#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
px4_root="${PX4_AUTOPILOT_ROOT:-"$workspace/../PX4-Autopilot"}"

if [[ ! -d "$px4_root/Tools/simulation/gz" ]]; then
    echo "PX4 tree not found: $px4_root" >&2
    exit 2
fi
if ! pgrep -f MicroXRCEAgent >/dev/null; then
    echo "MicroXRCEAgent is not running; start it before enabling flight." >&2
fi
echo "Safety: launch defaults to observation only and cannot arm."
echo "PX4: $px4_root"
echo "Run the overlay dry-run first:"
"$workspace/simulation/scripts/install_px4_overlay.sh" "$px4_root"
echo
echo "After reviewing, install with --apply, start PX4/QGC manually, then:"
echo "source /opt/ros/jazzy/setup.bash"
echo "source \"$workspace/install/setup.bash\""
echo "ros2 launch uav_vision qr_shelf_task.launch.py mode:=sitl \\"
echo "  target_qr_id:=7 enable_offboard:=true enable_auto_arm:=true"
