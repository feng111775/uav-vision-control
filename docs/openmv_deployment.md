# OpenMV H7 Plus终端部署与回滚

正式设备使用固定别名`/dev/dtask_openmv`、115200波特率。任何串口访问前必须用
`scripts/hardware/check_device_identity.py`核对VID、PID和设备私有序列号；禁止
按`/dev/ttyACM*`猜测设备，禁止连接Pixhawk。

## 文件和工具

正式板载文件包括`camera_config.py`、`detector.py`、`detector_fast.py`、`protocol.py`和`main.py`。
旧`thresholds.py`不参与正式算法，也不由部署工具复制。

```bash
python3 scripts/hardware/deploy_openmv.py --dry-run
python3 scripts/hardware/deploy_openmv.py
python3 scripts/hardware/openmv_repl.py --exec "import os; print(os.listdir('/flash'))"
python3 scripts/hardware/validate_openmv_live.py --duration 20
python3 scripts/hardware/test_visual_pipeline_pty.py
```

部署工具先把包含隐藏文件的完整磁盘复制到`~/openmv_backups`，再复制四文件、
执行`sync`和SHA256校验，最后用`udisksctl`安全卸载并软重启。只有显式传入
`--rollback BACKUP`才会回滚；不会更新固件、格式化或擦除文件系统。

## OpenMV v5.0启动语义

本机随OpenMV IDE安装的v5.0.0离线文档明确说明：`boot.py`在软重启执行，
`main.py`只在冷启动执行。因此Ctrl-D后直接出现REPL是正常行为，不能据此判断
`main.py`部署失败。部署后可先手动`exec(open('/flash/main.py').read(), {})`
诊断，再用安全整板复位或重新上电验证自动启动。复位前必须安全卸载USB磁盘。

## 当前性能边界

2026-07-30无目标实测显示，全帧Hough圆搜索是主要瓶颈。当前实现只在尚未锁定
目标时每8帧执行一次全帧圆搜索；七字段协议、圆环几何、十字规则和全部阈值
未改变。无目标平均输出仍低于15Hz，不能声称已达到最终性能。首次捕获延迟与
正样本效果必须等待真实目标制作完成后复测。

## 树莓派离线自启动

三个`scripts/deploy/*_vision_service.sh`脚本管理只读D题视觉链，不启动任务
控制器、Offboard控制器、自动解锁或自动起飞：

```bash
D_TASK_WORKSPACE="$HOME/px4_ros2_ws" \
  scripts/deploy/install_vision_service.sh --dry-run
scripts/deploy/install_vision_service.sh
scripts/deploy/check_vision_service.sh
scripts/deploy/uninstall_vision_service.sh
```

用户systemd服务不依赖网络，允许OpenMV晚插入，异常退出自动重启，并使用
journald限频。若`corn-pi.local`不可达，只能验证脚本语法和dry-run，不能声称
已在树莓派安装或完成冷启动验证。

真实目标尚未完成，以下项目均为“待真实目标制作完成后补测”：valid=1正样本、
中心位置变化、角度变化、遮挡恢复和移除目标后的LOST时间。
