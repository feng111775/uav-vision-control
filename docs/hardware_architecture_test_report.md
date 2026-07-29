# 第三阶段B1测试报告

测试日期：2026-07-29。起始提交：
`9b114737521de2a8123bc69efe98b9d8ef982a72`。

本阶段只验证软件架构：没有连接 OpenMV、小车链路、payload、Pixhawk 等真机，
没有启动 PX4，没有解锁。真实串口、波特率、IP、机构、阈值和尺寸均保持未配置。

## 安装与静态检查

- `setup.py` 安装 8 个 launch、9 个 YAML；隔离安装目录逐项可见。
- `package.xml` 声明 ROS 消息、launch、ament index、OpenCV、NumPy 和 pyserial
  运行依赖。
- `ros2 pkg executables uav_control` 列出 9 个入口；部署自检通过，且没有打开
  硬件。修正了自检脚本把 ROS 私有 `lib/uav_control` 目录误当作通用 `PATH`
  目录而产生的假阴性。
- `python3 -m compileall -q src/uav_control src/uav_vision`：通过。
- 全部 YAML 配置验证：通过。

## 隔离构建与全量测试

在 `/tmp/d-task-b1.*` 的独立 build/install/log 目录执行：

- `colcon build --packages-select uav_vision uav_control`：2 包通过。
- `colcon test --packages-select uav_vision uav_control`：
  `uav_vision` 109 passed；`uav_control` 158 passed、1 skipped。
- 汇总：**268 tests、0 errors、0 failures、1 skipped**。
- skip 是既有 OpenMV MicroPython 模块不能在 CPython 主机导入的硬件相关测试，
  不是本阶段倒退。
- 协议隔离定向烟雾：22 passed、36 deselected；覆盖 localhost UDP、PTY 串口及
  payload 协议/状态机。UDP 收到并解析 `progress=1`；PTY 在 115200 测试速率
  读回完整测试行，该速率不是任何真实设备配置。

## ROS localhost mock烟雾

所有 launch 均使用 `ROS_LOCALHOST_ONLY=1` 和独立 Domain ID；关闭前日志无
traceback、进程死亡或 ERROR。

- observe：6 个节点启动；`/system/status` 发布失鲜状态，safety 为 false；
  2 秒监听 VehicleCommand 为 0 条。
- bench disabled：11 个节点启动；car/payload 均为 `DISABLED`，safety 为
  false，2 秒监听 Offboard 为 0 条。
- bench mock：12 个节点启动；显式 mock 小车发布进度；一次 release 获得一次
  true ack，payload 状态为 `SUCCESS`、attempts=1。
- first_flight：仅 4 个集成/任务节点启动，无 car、payload、mock 或 SITL；
  safety 为 false，原因为 `OPERATOR_ENABLE_MISSING_OR_STALE`，Offboard 为
  0 条。
- competition drop：6 个节点启动，无 mock 或 SITL；car/payload 均为
  `DISABLED`，safety 为 false，原因为
  `OPERATOR_ENABLE_MISSING_OR_STALE`，payload ack 为 0 条。

## 结论

软件构建、两包全量测试、协议隔离和正式入口 mock 隔离均通过。物理结论：无；
本报告不能替代拆桨台架、真机链路或飞行验证。
