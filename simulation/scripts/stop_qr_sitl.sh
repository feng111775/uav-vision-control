#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
log_root="${QR_SITL_LOG_DIR:-"$workspace/test_results/qr_sitl_gui"}"
pid_file="$log_root/pids"

if [[ -f "$pid_file" ]]; then
    mapfile -t pids <"$pid_file"
    for pid in "${pids[@]}"; do
        [[ "$pid" =~ ^[0-9]+$ ]] && kill -INT "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${pids[@]}"; do
        [[ "$pid" =~ ^[0-9]+$ ]] && kill -TERM "$pid" 2>/dev/null || true
    done
    : >"$pid_file"
fi

pkill -f 'rqt_image_view.*/vision/qr/debug_image' 2>/dev/null || true
if [[ "${1:-}" == "--all-conflicts" ]]; then
    pkill -f 'px4.*gz_x500' 2>/dev/null || true
    pkill -f 'gz sim.*qr_shelf_world' 2>/dev/null || true
    pkill -f 'MicroXRCEAgent udp4 -p 8888' 2>/dev/null || true
    pkill -f 'qr_shelf_task.launch.py' 2>/dev/null || true
fi

for _ in $(seq 1 20); do
    ss -lun | grep -q ':8888 ' || {
        echo "QR SITL stopped; UDP 8888 released"; exit 0; }
    sleep .1
done
echo "UDP 8888 is still in use" >&2
exit 1
