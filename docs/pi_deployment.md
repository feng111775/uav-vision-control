# 树莓派部署

先运行 `scripts/pi/check_environment.py`；它只报告，不升级系统。使用
`scripts/pi/build_workspace.sh --test` 在 `/tmp` 隔离构建，不清理历史目录。
`deploy_workspace.sh --target ...` 默认 dry-run，只选择 Git 跟踪文件并排除
build/install/log、datasets、models 和 `.git`；加 `--apply` 前目标 Git 工作树
必须干净。部署清单由 SHA256 支持审计。

安装后加载 ROS 与部署 install，再运行 `scripts/pi/verify_install.py`。DDS 串口
正式默认值为 `/dev/ttyAMA0`、921600；对应 systemd 环境变量为
`XRCE_TRANSPORT=serial`、`XRCE_DEVICE=/dev/ttyAMA0`、`XRCE_BAUDRATE=921600`。
手动命令等价于 `MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 -v 4`。
支持包用
`collect_support_bundle.sh` 本地生成，内容排除数据集、密钥、Wi-Fi/SSH 凭据，
且不会上传。

`deploy/systemd` 仅是模板：不得自动安装、enable 或 start。Agent、视觉和只读
健康允许失败重启；observe 不自动重启；没有自动解锁、开机飞行或 competition
任务服务。systemd 模板中的 DDS 串口值是已验证链路的示例默认值，安装或启用
服务前仍需人工审核设备节点与权限。
