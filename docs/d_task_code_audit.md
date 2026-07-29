# 2026 D题第一阶段代码审计

审计日期：2026-07-29。范围为唯一工作空间 `/home/xixi/px4_ros2_ws` 及用户主目录
中同名目录。扫描到的其他副本仅存在于本工作空间的 `build_*`、`install_*`、
`log_*` 生成目录，均未迁移。未发现第二个可作为源码来源的旧工作空间。

| 文件/目录 | 当前功能 | 复用/版本判断 | 红色旧方案/重复 | 处置与最终位置 | 理由 |
|---|---|---|---|---|---|
| `uav_vision/h7_bridge_node.py` | H7 串口桥 | 节点结构可复用；协议需更新 | 旧 `TARGET` 九字段 | 原位改为 `D_TARGET` 七字段 | 保留重连、异常隔离和限频日志 |
| `uav_vision/target_filter_node.py` | EMA、确认和丢失保持 | 状态机可复用；schema/角度需更新 | 旧九字段 | 原位更新 | 90°周期角度不能线性平均 |
| `uav_vision/fake_h7_node.py` | 假数据源 | 发布结构可复用 | 旧红色矩形格式，且唯一假源 | 原位替换 | 避免创建重复节点 |
| `uav_vision/detection.py` | 旧九字段矩形 schema | 不可供 D 题链路复用 | legacy；仍被旧 Gazebo/视觉伺服引用 | 保留原位，不进 D 题 launch | 下一阶段 SITL 兼容入口尚未替换，不能安全删除 |
| `gazebo_red_target_detector_node.py` | Gazebo 红色色块检测 | D 题正式检测不可复用 | legacy | 保留原位，不进 D 题 launch | 后续 SITL 入口可能需要，当前测试仍引用 |
| `visual_servo_node.py` | 旧视觉误差转换 | D题职责由 landing error 替代 | legacy/功能近似重复 | 保留原位，不进 D 题 launch | 旧仿真与测试仍引用；本阶段不得跨入 PX4 控制 |
| `camera_selector_node.py` | 双相机选择 | 本阶段不直接使用 | legacy | 保留原位 | 下一阶段仿真可能需要 |
| `gazebo_vision.launch.py`, `dual_camera_simulation.launch.py` | 旧仿真入口 | 本阶段不直接使用 | legacy | 保留原位 | 后续 SITL 边界明确要求保留可能入口 |
| `red_target_hardware.launch.py` | 红色目标硬件/控制链 | 不可作为正式 D 题入口 | legacy，且会联动控制 | 保留但不启动 | 当前测试仍引用；删除会破坏兼容测试 |
| 三个旧 YAML | 旧仿真/硬件参数 | D 题正式链路不复用 | legacy | 保留原位 | 配套旧入口仍存在 |
| `openmv_h7plus/camera_config.py` | CSI QVGA 初始化 | 直接复用 | 否 | 原位保留 | 满足保留摄像头初始化要求 |
| `openmv_h7plus/protocol.py` | USB CDC 文本输出 | CDC 可复用，协议需更新 | 旧 `TARGET` | 原位改为 `D_TARGET` | 不复制/移动硬件目录 |
| `openmv_h7plus/detector.py`, `thresholds.py` | 红色色块检测 | 仅硬件通信测试可复用 | legacy 红色色块 | 原位保留并显式适配 D 字段 | 不伪造完整同心圆算法 |
| `openmv_h7plus/main.py` | 摄像和发送循环 | 直接复用框架 | detector 为 legacy | 原位保留 | 单帧异常和 USB 断开不退出 |
| `src/uav_control/**` | PX4 Offboard/状态节点 | 本阶段只读 | 非视觉模块 | 不修改 | PX4 v1.16 接口已另行记录；禁止自动解锁 |
| `src/pi_camera_vision/**` | 树莓派相机旧红色检测 | 本阶段只读 | legacy 红色方案 | 不修改 | 不跨包、不修改用户指定边界 |

## 新增必要项

- `d_task_schema.py`：唯一索引、长度、校验、无效值和90°角度工具。
- `target_predictor_node.py`：接收时间速度估计、限幅、常速度预测和年龄。
- `landing_error_node.py`：像素和归一化误差，不生成虚假米制值。
- `vision_dashboard_node.py`：无相机画布和只读状态文本。
- `d_task_vision.yaml` 与 `d_task_vision.launch.py`：唯一正式第一阶段链路。
- schema、滤波、预测、误差和串口协议 pytest。

## 删除结论

本阶段没有文件同时满足全部 `git rm` 条件，因此未删除源码。所有 legacy 文件均已
从新的正式 D 题 launch 隔离；待第二阶段 SITL 替换完成、旧测试和 setup 引用解除后
再复审删除。
