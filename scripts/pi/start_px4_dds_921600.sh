#!/usr/bin/env bash
set -euo pipefail

workspace="${D_TASK_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
env_file="${PX4_DDS_ENV_FILE:-$workspace/scripts/pi/px4_dds_921600.env}"
source "$env_file"
: "${PX4_DDS_DEVICE:?PX4_DDS_DEVICE is required}"
: "${PX4_DDS_BAUDRATE:?PX4_DDS_BAUDRATE is required}"
[[ "$PX4_DDS_BAUDRATE" == 921600 ]] || { echo 'DDS baudrate must remain 921600' >&2; exit 2; }
[[ -e "$PX4_DDS_DEVICE" ]] || { echo "DDS device missing: $PX4_DDS_DEVICE" >&2; exit 1; }
[[ "$(readlink -f "$PX4_DDS_DEVICE")" != "$(readlink -f /dev/dtask_openmv 2>/dev/null || true)" ]] || {
  echo 'DDS device must not be the OpenMV serial device' >&2; exit 2; }
agent="$(command -v MicroXRCEAgent || true)"
[[ -n "$agent" ]] || { echo 'MicroXRCEAgent not found' >&2; exit 1; }
mapfile -t existing < <(pgrep -af 'MicroXRCEAgent serial' || true)
(( ${#existing[@]} == 0 )) || { printf 'DDS agent already running:\n%s\n' "${existing[*]}" >&2; exit 3; }
printf 'MicroXRCEAgent version=%s device=%s baudrate=%s\n' "${PX4_DDS_AGENT_VERSION:-unknown}" "$PX4_DDS_DEVICE" "$PX4_DDS_BAUDRATE"
if [[ "${PX4_DDS_DRY_RUN:-false}" == true ]]; then exit 0; fi
exec "$agent" serial --dev "$PX4_DDS_DEVICE" -b "$PX4_DDS_BAUDRATE"
