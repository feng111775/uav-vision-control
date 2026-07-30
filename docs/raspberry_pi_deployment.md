# Raspberry Pi deployment

Use scripts in `scripts/deploy` from `/home/a-corn/px4_ros2_ws`. They use `git pull --ff-only`, refuse dirty worktrees, source Jazzy, and make no network download at runtime. Check `/dev/dtask_openmv` before launching. Udev installation is deliberately manual/system-admin controlled.
