# First Task Real Aircraft Runbook

状态：`READY_FOR_FIRST_TASK_HARDWARE_ACCEPTANCE`

基线：

- PX4 v1.16.0 / `PX4_FMU_V6C`
- `px4_msgs` `release/1.16`
- ROS 2 Jazzy / Ubuntu 24.04
- Raspberry Pi 4B aarch64
- OpenMV H7 Plus，固件 5.0.0，MicroPython 1.28
- OpenMV `/dev/dtask_openmv` at 115200
- DDS transport at 921600

正式视觉链路：

`D_TARGET_V2 -> h7_bridge_node -> /vision/internal/h7/raw -> vision_interface_node -> /vision/target/tracked + /vision/landing_error + /vision/health`

正式飞控只订阅结构化 V2 消息；旧 `Float32MultiArray` 链路不得进入正式 launch。

Profile：

- `readonly_bench`: control、auto-arm、payload release 全部关闭，servo dry-run。
- `no_prop_control`: 可验证 setpoint 和坐标方向，禁止自动解锁和真实舵机。
- `real_competition`: 只有通过 preflight、180 秒视觉审计、OpenMV 拔插恢复、ESP32 联网、GPIO18 台架和无桨方向验收后才能使用。

任务状态机：

`WAIT_START -> TAKEOFF -> HOVER_150CM -> HOVER_3S -> SEARCH_CAR -> VISION_FOLLOW -> ALIGN_FOR_DROP -> PAYLOAD_RELEASE -> WAIT_RELEASE_ACK -> RETURN_HOME -> FINAL_LAND -> COMPLETE`

只有检测到 landed 且 Disarmed 后才能进入 `COMPLETE`。90 秒超时进入失败路径，不得虚报完成。

ESP32 使用 `car_udp_real.yaml`，监听 `0.0.0.0`，不在代码中硬编码固定 IP。GPIO18 由 `servo_real.yaml` 配置，真实动作由 `dry_run=false` 显式启用。

`ready_for_closed_loop` 默认 false。它要求 V2 数据新鲜、序列连续、协议无错误、性能预热完成、相机安装和坐标映射已人工验证；硬故障立即关闭，软性能故障使用连续失败计数。

硬件验收顺序：

1. 只读树莓派视觉链路 180 秒。
2. OpenMV 真实拔插并确认恢复。
3. ESP32 真实 UDP 启动消息、非法来源拒绝、重复消息去重。
4. GPIO18 舵机台架一次投放和 ACK。
5. 无桨 PX4/DDS/方向和视觉失效撤销。
6. 低高度短时闭环。
7. 返航、最终降落、landed 和 Disarmed。

