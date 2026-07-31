# 视觉 V2 与飞控控制组接口交接

## 1. 当前分支用途

本分支用于视觉组与控制组进行正式 V2 接口联合开发和桌面联调。

分支：

integration/vision-v2-control-handoff

当前状态：

READY_FOR_CAR_LINK_AND_SITL_TRIGGER_TEST

尚未达到：

- READY_FOR_CLOSED_LOOP
- READY_FOR_REAL_TRACKING
- READY_FOR_FLIGHT
- READY_FOR_FULL_MISSION

## 2. 当前已经完成

控制组已完成：

- 小车 START UDP 网关；
- STM32 数据帧 ASCII/XOR 校验；
- START 去重和会话保护；
- communication_only 安全模式；
- /car/mission_start 接入任务状态机；
- 非视觉 1.5 m 起飞；
- 悬停 3 秒；
- 比赛小车沿赛道运行速度按 `0.1 m/s` 记录；该值不是无人机固定机体系
  速度前馈，伴飞仍由实时视觉误差闭环完成；
- 投放对准连续确认时间为 `0.4 s`；按小车速度计算为 `0.1 × 0.4 = 0.04 m`
  （4 cm），但确认期间无人机仍持续跟随和修正，不能理解为固定悬停等待；
- 自动降落和安全结束；
- enable_visual_follow 状态机硬门控；
- target_loss_abort_seconds 参数生效；
- `vision_adapter_mode=legacy_array` 统一适配层；
- simulation-only `mock_vision_node` 场景发布器；
- fail-closed dry-run/GPIO 投放后端接口；
- 自动测试和 ROS 2 构建通过。

非视觉流程：

WAIT_START
→ PRESTREAM
→ TAKEOFF
→ HOVER_150CM
→ HOVER_3S
→ FINAL_LAND
→ COMPLETE

## 3. 当前控制侧视觉接口

当前控制代码暂时仍使用旧接口：

/vision/target/tracked
std_msgs/msg/Float32MultiArray

/vision/landing_error
std_msgs/msg/Float32MultiArray

这是等待视觉组正式 V2 包期间保留的旧接口，不代表最终接口。

控制器通过 `vision_contract.LegacyVisionAdapter` 将两个数组转换为统一
`VisionObservation`；任务状态机不读取数组下标。`vision_receive_timeout_seconds`
和 `vision_source_age_limit_seconds` 默认均为 0.3 s，`vision_health_required`
默认关闭，打开时可暂时使用明确标注为 legacy 的 `std_msgs/msg/Bool`
`/vision/health_legacy_bool`。

`ALIGN_FOR_DROP` 不使用固定延时投放：目标、健康、时间戳、confidence 和
X/Y 误差必须持续满足条件 0.4 s；任一帧失效都会从零重新计时。对准期间仍
输出受现有最大速度限制的实时视觉修正。

控制侧不会要求视觉正式 V2 话题退回 Float32MultiArray。

## 4. 视觉组正式 V2 接口

计划迁移为：

/vision/target/tracked
uav_interfaces/msg/TargetObservation

/vision/landing_error
uav_interfaces/msg/LandingError

/vision/health
uav_interfaces/msg/VisionHealth

视觉组上传代码后，需要提供完整源码包：

uav_interfaces/
├── package.xml
├── CMakeLists.txt
└── msg/
    ├── TargetObservation.msg
    ├── LandingError.msg
    └── VisionHealth.msg

不得只提供字段文字说明，必须提交完整 ROS 2 消息包源码。

交付字段和安装信息见 `docs/VISION_V2_DELIVERY_CHECKLIST.md`。V2 到来后只
替换适配器和配置，不能让视觉节点发布 ARM、Offboard 或 PX4 输入。

## 5. 正式视觉代码接入路径

控制工作区示例：`${PX4_ROS2_WS:-$HOME/px4_ros2_ws}`

控制源码位于当前仓库的 `src/uav_control`；不要在 Python 代码中硬编码本机路径。

视觉接口包计划放置：`${PX4_ROS2_WS}/src/uav_interfaces`

控制主节点：

src/uav_control/uav_control/mission_controller_node.py

状态机：

src/uav_control/uav_control/mission_logic.py

视觉协议旧适配：

src/uav_control/uav_control/vision_contract.py

## 6. 视觉阶段

任务状态机包含：

- SEARCH_CAR
- VISION_FOLLOW
- ALIGN_FOR_DROP
- ALIGN_PLATFORM
- DYNAMIC_DESCENT_HIGH
- DYNAMIC_DESCENT_NEAR

视觉组与控制组完成正式接口联合验收前，这些阶段不得用于真机自动飞行。

## 7. 控制侧预期安全条件

正式视觉闭环至少需要：

- enable_visual_follow=true；
- /vision/health 未超时；
- ready_for_closed_loop=true；
- camera_open=true；
- algorithm_alive=true；
- protocol_ok=true；
- capture_stamp_valid=true；
- measurement_valid或valid=true；
- confirmed=true；
- predicted=false；
- confidence达到双方确定的阈值；
- 控制字段不是NaN或Inf；
- 本地接收年龄不超过300 ms；
- 图像采集时间年龄不超过300 ms。

任一条件不满足时，控制器不得继续使用旧坐标进行自动追踪、投放对准或动态下降。

## 8. 坐标方向待联合确认

视觉组当前定义：

error_x_norm > 0：目标位于画面右侧；
error_x_norm < 0：目标位于画面左侧；
error_y_norm > 0：目标位于画面下方；
error_y_norm < 0：目标位于画面上方。

控制组不会在未完成实物摄像头安装方向测试前，假定画面上方等于机头前方。

联合验收必须确认：

- 摄像头安装旋转方向；
- 是否镜像；
- error_x对应机体前后还是左右；
- error_y对应机体前后还是左右；
- 正负方向到PX4 NED速度指令的映射。

## 9. 视觉组代码合入后的流程

1. 导入正式 uav_interfaces；
2. 单独构建 uav_interfaces；
3. 修改控制节点结构化消息订阅；
4. 接入 /vision/health；
5. 更新视觉模拟节点；
6. 完成NaN、断流、超时和健康门测试；
7. 桌面联合验收；
8. 静止目标无桨验收；
9. 低高度静止目标飞行；
10. 移动小车跟随；
11. 投放和返航完整验收。

## 10. 构建命令

cd "${PX4_ROS2_WS:-$HOME/px4_ros2_ws}"

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select uav_control

source install/setup.bash

colcon test \
  --packages-select uav_control

colcon test-result --verbose

部署前只读检查（不启动控制节点）：

```bash
source /opt/ros/jazzy/setup.bash
bash "${UAV_CONTROL_SRC:-$PWD/src/uav_control}/scripts/check_deployment.sh"
```

树莓派部署顺序：先确认 ROS 2 Jazzy、`px4_msgs` 和 DDS Agent 环境，再构建并
source 工作区；启动顺序必须是 PX4/DDS 就绪、任务控制器进入 `WAIT_START`、
视觉（仅联调时）和车机网关，最后才允许实体小车 START。此仓库不安装或启用
systemd 服务，也不在部署检查脚本中启动飞控。

## 11. 当前测试结果

当前控制组报告：

- uav_control构建通过；
- Python测试469 passed、1 skipped；
- colcon test无failure和error；
- 未进行视觉V2真机闭环；
- 未进行完整真机任务。
