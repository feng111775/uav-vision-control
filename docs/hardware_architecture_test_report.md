# 第三阶段B1测试报告

本阶段仅验证软件架构，无真实硬件、无解锁、无PX4飞行。起始提交：
`9b114737521de2a8123bc69efe98b9d8ef982a72`。实际构建、测试和烟雾结果在
最终回归后填写；真实串口、波特率、IP、机构、阈值和尺寸均保持未配置。

覆盖范围：CRC16小车协议、uint16回绕/单调进度、localhost UDP、伪终端串口、
payload边沿/sequence/有限重试、disabled行为、健康失鲜、模式化安全门控、
competition mock隔离、部署清单与dry-run、systemd无自动解锁边界。

隔离构建第一次功能回归结果为 268 tests、0 errors、0 failures、1 skipped；
其中新增硬件架构测试 66 项。skip 是既有 OpenMV MicroPython 模块不能在
CPython 主机导入的硬件相关测试，不是本阶段倒退。localhost UDP 收到并解析
progress=1；pseudo-terminal 在 115200 测试速率完整读回测试行（该数值只是
PTY 测试参数，不是任何真实设备配置）。

烟雾测试结果：

- observe：六节点 clean 启动，`/system/status` 与 safety false 持续发布；
  VehicleCommand 只有发布端点但 2 秒监听为 0 条消息。
- bench disabled：正式视觉 fake 链和集成节点 clean 启动；car 显示 DISABLED，
  safety false，Offboard topic 无消息。
- bench mock：mock car/payload 只在显式参数下启动；一次 release 获得一次
  true ack。
- first_flight：controller 参数实测 `enable_control=False`，无 mock/car/payload
  节点。
- competition drop：无 mock/SITL 节点，car/payload disabled，safety reason
  为 `OPERATOR_ENABLE_MISSING_OR_STALE`，未产生 payload ack。

物理结论：无。未连接 OpenMV、小车无线链路、payload 或 Pixhawk 真机。
