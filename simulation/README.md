# PX4 SITL overlay

`px4_overlay/` 保留相对于 `PX4-Autopilot` 根目录的原始路径，只包含本项目新增
的双摄机型、红色目标世界和 airframe，不包含完整 PX4 仓库或编译产物。

安装到一个已存在的 PX4 v1.17 源码树：

```bash
./simulation/scripts/install_px4_overlay.sh ~/PX4-Autopilot
```

脚本会先验证目标目录并列出待安装文件，然后复制 overlay；它不会删除任何 PX4
文件。`CMakeLists.txt` 中缺少 airframe 项时，只添加
`4022_gz_x500_downward_camera` 一行。
