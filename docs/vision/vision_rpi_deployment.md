# Raspberry Pi 离线视觉部署

依赖：ROS 2 Jazzy、Python 3、`rclpy`、`pyserial`、`uav_interfaces`、`uav_vision`、udev 稳定别名 `/dev/dtask_openmv` 和当前用户 `dialout` 权限。

执行 `scripts/deploy/audit_pi_vision_deps.sh` 审计依赖，`scripts/deploy/deploy_pi_vision.sh` 选择性构建并生成只读 systemd 模板。安装脚本默认禁用服务，不自动启动；服务只执行 `h7_v2_readonly.launch.py`，不执行 PX4 Agent、uav_control 或任何控制节点。

人工验收时可执行 `scripts/deploy/run_pi_readonly_acceptance.sh`。卸载使用现有 `scripts/deploy/uninstall_vision_service.sh`。工作区由 `D_TASK_WORKSPACE` 或脚本自身路径解析，不假设用户名。
