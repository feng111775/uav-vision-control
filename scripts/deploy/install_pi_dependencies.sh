#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/jazzy/setup.bash
python3 -c 'import serial' || { echo 'Install python3-serial from your offline apt mirror before deployment.'; exit 2; }
echo 'Offline dependency check passed; this script intentionally downloads nothing.'
