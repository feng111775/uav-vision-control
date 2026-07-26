#!/usr/bin/env bash
set -euo pipefail

echo "Architecture: $(uname -m)"
echo "CPUs: $(nproc)"
free -h
/usr/bin/python3 - <<'PY'
import cv2
import platform
print("Python:", platform.python_version())
print("OpenCV:", cv2.__version__)
print("Coral runtime:", end=" ")
try:
    import pycoral
    print(getattr(pycoral, "__version__", "installed"))
except ImportError:
    print("not installed")
PY
for device in /dev/video* /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$device" ]] && echo "Device: $device"
done
true
