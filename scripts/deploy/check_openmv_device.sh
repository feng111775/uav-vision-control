#!/usr/bin/env bash
set -euo pipefail
test -c /dev/dtask_openmv || { echo 'FAIL: /dev/dtask_openmv missing'; exit 1; }
python3 -c 'import serial; print("pyserial OK")'
echo 'OpenMV device and Python serial dependency OK'
