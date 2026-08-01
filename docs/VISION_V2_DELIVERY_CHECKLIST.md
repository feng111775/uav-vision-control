# 视觉 V2 交付清单

正式 V2 尚未接入。当前控制侧只保留 `legacy_array` 适配器：
`std_msgs/msg/Float32MultiArray` 仅是兼容接口，不是最终 V2 协议。

视觉组交付前请提供：

1. ROS 2 包名、可执行程序名和启动 Launch；
2. target、health 话题名称和完整消息类型；
3. `ros2 interface show` 输出及每个字段含义、单位、坐标系；
4. x/y 误差正方向、valid 定义、confidence 范围和时间戳来源；
5. 发布频率、无目标发布方式、节点异常时 health 行为；
6. 相机分辨率、视场角、目标 ID/类别及可用尺度或深度字段；
7. 示例消息或录包，以及树莓派依赖和安装方式。

控制侧需要确认：

比赛小车运行速度按 `0.1 m/s` 处理。它描述小车沿赛道的速度，不应直接映射
为无人机固定坐标轴速度；无人机伴飞必须继续使用实时视觉误差。

投放对准必须连续满足全部条件 `0.4 s`。`0.1 × 0.4 = 0.04 m` 只是小车
在确认窗口内的理论路程，确认期间无人机仍持续跟随和修正。

- `/vision/target/tracked` 与 `/vision/landing_error` 的 V2 替代关系；
- `/vision/health` 的正式消息和健康语义；
- valid、confirmed、predicted、confidence 和 source timestamp 的定义；
- 丢失目标与视觉进程离线的区分；
- 坐标轴和正负方向映射。

V2 到来后，只应新增/替换 `uav_control/vision_contract.py` 中的适配器和
对应配置；不要修改任务状态机，也不要让视觉节点发布 ARM、Offboard 或
任何 PX4 输入。任务状态机和飞行安全仍由控制组负责。

桌面联调可选用明确的 `mock_vision_node`：

```bash
ros2 launch uav_control mock_vision.launch.py scenario:=aligned
```

它仅在 `simulation_mode=true` 下运行，不打开相机、不连接 PX4，也不驱动舵机。
