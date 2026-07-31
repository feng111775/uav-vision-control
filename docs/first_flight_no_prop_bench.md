# 无桨台架

使用 `first_flight_bench.launch.py`，不装桨、不启动视觉或完整 D 题任务。
确认 `/fmu/out/*` 持续更新、`/uav/safety/ready` 仅在检查通过时为 true，调用
`ros2 service call /real_practise/start std_srvs/srv/Trigger {}` 后观察预发送、
Offboard 请求、VehicleCommandAck 和人工接管路径。任何 DENIED、FAILED、超时或重复发布者都必须中止。
