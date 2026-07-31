# Raspberry Pi deployment

Use scripts in `scripts/deploy` from `/home/a-corn/px4_ros2_ws`. They use `git pull --ff-only`, refuse dirty worktrees, source Jazzy, and make no network download at runtime. Check `/dev/dtask_openmv` before launching. Udev installation is deliberately manual/system-admin controlled.

For the verified PX4 GPS2 DDS serial link, use `/dev/ttyAMA0` at 921600 baud.
The formal Agent command is:

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 -v 4
```

PX4 must use GPS2 (`UXRCE_DDS_CFG=202`, `GPS_2_CONFIG=0`) and the same baud
rate; do not change the OpenMV serial setting.
