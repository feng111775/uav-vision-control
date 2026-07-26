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

`simulation/scripts/run_qr_sitl_headless.sh` 会启动或复用 Agent、协议级 GCS
heartbeat、PX4/Gazebo 和 ROS launch，并把原始日志放在已忽略的
`test_results/`。默认仍不启用 Offboard 或自动解锁。

安装器默认 dry-run、校验 v1.17、覆盖前备份。headless 可用协议 heartbeat
满足正常数据链检查，不修改 PX4 飞检参数。显式设置
`mode:=sitl enable_offboard:=true enable_auto_arm:=true` 且
`simulation_mode:=true` 才可能解锁。
