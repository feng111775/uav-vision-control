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
- 自动降落和安全结束；
- enable_visual_follow 状态机硬门控；
- target_loss_abort_seconds 参数生效；
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

## 5. 正式视觉代码接入路径

控制工作区：

/home/a-corn/px4_ros2_ws

控制源码：

/home/a-corn/XTU-uav-vision-control-git/src/uav_control

视觉接口包计划放置：

/home/a-corn/px4_ros2_ws/src/uav_interfaces

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

cd /home/a-corn/px4_ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select uav_control

source install/setup.bash

colcon test \
  --packages-select uav_control

colcon test-result --verbose

## 11. 当前测试结果

当前控制组报告：

- uav_control构建通过；
- Python测试469 passed、1 skipped；
- colcon test无failure和error；
- 未进行视觉V2真机闭环；
- 未进行完整真机任务。
