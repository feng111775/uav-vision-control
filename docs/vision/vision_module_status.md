# Vision module status

当前交付链路是只读 V2：OpenMV `D_TARGET_V2` → `h7_bridge_node` → `/vision/internal/h7/raw` → `vision_interface_node` → `/vision/target/tracked`、`/vision/landing_error`、`/vision/health`，另有 `/vision/h7/status`。

本模块不发布任何 `/fmu/in/*`，`ready_for_closed_loop` 固定为 `false`。`ready_for_mission` 只在相机、帧接收、算法、协议、20 Hz 新帧和 300 ms 新鲜度条件同时满足时为真；这不表示允许闭环飞行。

验收入口：`src/uav_vision/launch/h7_v2_readonly.launch.py`。服务模板默认禁用，脚本不会启动 PX4、MicroXRCEAgent、uav_control 或旧滤波/控制节点。
