# D题第二阶段代码审计

日期：2026-07-29，基线提交 `7a76618a22e7caf5b16659dc95f3db5ed306b492`。

## 正式保留和替代关系

| 旧项 | 引用情况 | 第二阶段处置 | 理由 |
|---|---|---|---|
| `uav_control/offboard_control.py` | 旧setup入口和旧文档 | 从setup正式入口移除，文件标记legacy保留 | 旧测试直接导入；贸然删除会破坏回归，且不得作为正式飞行入口 |
| `vision_offboard_controller.py` | 两个旧launch、旧测试、红色硬件launch | 从setup正式入口移除，保留legacy | 新控制器已替代正式职责，但历史安全测试仍依赖 |
| `vehicle_status_listener.py` | 独立只读入口 | 保留 | 唯一只读诊断工具，不发布控制命令 |
| `visual_servo_node.py` | 旧Gazebo/双相机/硬件launch和测试 | 保留legacy，不进入D题launch | `uav_control.visual_guidance`已替代正式职责；旧引用尚未解除 |
| 红色Gazebo节点、launch、YAML | 多个旧测试/文档 | 保留legacy，不进入D题launch | 删除条件不成立；OpenMV legacy通信代码按要求不删除 |
| 双相机选择节点和launch | 旧SITL测试 | 保留legacy | 仍有测试引用，且不进入新链路 |

旧 `uav_vision/package.xml` 对 `uav_control` 的exec依赖已移除，消除与正式
`uav_control → uav_vision(schema)` 的循环。跨包旧launch因此只作为legacy源码保留，
不再定义正式包依赖关系。

## 新正式链路

`mission_controller_node` 是唯一正式PX4发布者；`mission_dashboard_node`只读。
`d_task_sitl_scenario_node`仅由 `d_task_sitl.launch.py` 启动，不进入competition或首飞配置。
第一阶段schema、滤波、预测、误差和视觉dashboard原样复用。

## 删除结论

没有文件满足“所有引用解除、测试替代完成、下一阶段无用途”的全部条件，本阶段未删除
源码。旧 `offboard_control` 和 `vision_offboard_controller` 已从setup入口移除，不能再被
当成安装后的正式可执行程序。
