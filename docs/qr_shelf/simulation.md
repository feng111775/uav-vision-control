# Gazebo / PX4 SITL

`qr_shelf_world.sdf` 有 4×6、0.19 m 的真实贴图 visual、白色背板、碰撞墙和
下视红色降落区。overlay 保持 PX4 相对目录，不复制 PX4 仓库。

```bash
python3 simulation/scripts/generate_qr_shelf_assets.py
simulation/scripts/install_px4_overlay.sh ../PX4-Autopilot
# 核对列表后才执行：
simulation/scripts/install_px4_overlay.sh ../PX4-Autopilot --apply
simulation/scripts/check_and_run_qr_sitl.sh
```

安装器默认 dry-run、校验 v1.17、覆盖前备份。完整飞行要求人工 QGroundControl
连接和 Gazebo GUI，因此本次自动验证不宣称已飞行。显式设置
`mode:=sitl enable_offboard:=true enable_auto_arm:=true` 且
`simulation_mode:=true` 才可能解锁。
