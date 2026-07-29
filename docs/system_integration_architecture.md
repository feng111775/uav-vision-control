# D题硬件集成架构

```text
OpenMV → uav_vision → tracked/landing_error ┐
car serial/UDP → car_link_bridge ───────────┤
mission_controller → release → payload_bridge│
PX4 DDS ↔ mission_controller ────────────────┼→ health → safety_gate → ready
mission/vision/PX4/link topics ──────────────┘           (只读门控)
```

视觉字段和任务/PX4核心状态机保持冻结。小车桥只接收 A/B/C/D/A 状态；payload
桥只执行任务控制器发出的单次事件；health 不把“目标无效”当成硬件故障；
safety gate 不发布 PX4 命令，也不接管正在运行的任务。

所有硬件适配器默认 `disabled`：节点可发布 DISABLED 诊断，但不打开设备、
不发送数据、不生成成功 ack。真实端口、波特率、IP、尺寸和阈值位于
`docs/hardware_values_template.yaml` 的待填项，缺失时安全拒绝 ACTIVE/READY。

四层入口：

- `d_task_observe.launch.py`：只读，control/auto-arm 强制关闭。
- `d_task_hardware_bench.launch.py`：无桨台架，默认全 disabled，可显式 mock。
- `d_task_first_flight.launch.py`：唯一 1 m 首飞路径，默认控制关闭且无 mock。
- `d_task_competition.launch.py`：drop/dynamic_land，未配置硬件时不 READY，
  永不启动 mock/SITL/legacy。
