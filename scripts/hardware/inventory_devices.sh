#!/usr/bin/env bash
set -u
lsusb
for device in /dev/ttyACM* /dev/ttyUSB*; do
  [ -e "$device" ] || continue
  ls -l "$device"
  udevadm info --query=property --name="$device" | \
    grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_VENDOR=|ID_MODEL=|ID_USB_DRIVER=|DEVNAME=)' || true
  test -r "$device" && echo "READABLE=yes" || echo "READABLE=no"
  test -w "$device" && echo "WRITABLE=yes" || echo "WRITABLE=no"
done
