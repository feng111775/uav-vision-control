#!/usr/bin/env bash
set -euo pipefail
transport="serial"
device="/dev/ttyAMA0"
baudrate="921600"
port=""
dry_run=false
while (($#)); do
  case "$1" in
    --transport) transport="$2"; shift 2;;
    --device) device="$2"; shift 2;;
    --baudrate) baudrate="$2"; shift 2;;
    --port) port="$2"; shift 2;;
    --dry-run) dry_run=true; shift;;
    *) echo "unknown argument: $1" >&2; exit 2;;
  esac
done
agent="$(command -v MicroXRCEAgent || true)"
[ -n "$agent" ] || { echo "MicroXRCEAgent not found" >&2; exit 1; }
"$agent" --help >/dev/null 2>&1 || true
case "$transport" in
  udp4)
    [ -n "$port" ] || { echo "--port is required for udp4" >&2; exit 2; }
    command=("$agent" udp4 -p "$port")
    ;;
  serial)
    [ -n "$device" ] && [ -n "$baudrate" ] || {
      echo "--device and --baudrate are required for serial" >&2; exit 2; }
    [ -e "$device" ] || { echo "device does not exist: $device" >&2; exit 1; }
    command=("$agent" serial --dev "$device" -b "$baudrate" -v 4)
    ;;
  *) echo "--transport must be udp4 or serial" >&2; exit 2;;
esac
printf 'command:'
printf ' %q' "${command[@]}"
printf '\n'
$dry_run || exec "${command[@]}"
