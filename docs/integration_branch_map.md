# 2026 电赛 D 题集成分支与目录映射

## 集成基线

- 集成分支：`integration/d-task-final`
- 起始提交：`a5978ef65ec004b79c7ca7cb41011951adfdfe85`
- 起始功能分支：`feature/d-task-openmv-live-validation`
- 最终树莓派工作区：`~/px4_ros2_ws`

本分支以已验证视觉提交为起点。建立骨架时不合并主工作区内尚未提交的
小车/仿真修改，也不合并任何未确认的飞控、地面站或联合仿真分支。

## 模块与预定功能分支

| 模块 | 目录 | 预定来源 | 当前状态 |
|---|---|---|---|
| OpenMV H7 Plus | `openmv_h7plus` | `feature/d-task-openmv-live-validation` | 已包含已验证基线 |
| ROS 视觉 | `src/uav_vision` | `feature/d-task-openmv-live-validation` | 已包含已验证基线 |
| PX4 任务控制 | `src/uav_control` | 待负责人提供确认分支和提交 | 仅保留起始提交已有版本 |
| 小车控制 | `src/car_control`、`ground_vehicle` | 待负责人提供确认分支和提交 | 未合并主工作区未提交修改 |
| 联合仿真 | `src/d_system_sim` | 待负责人提供确认分支和提交 | 未合并主工作区未提交修改 |
| 离线地面站 | `src/ground_station`、`ground_station` | 待负责人提供确认分支和提交 | 仅保留起始提交已有版本 |
| 最终启动 | `src/d_task_bringup` | 集成完成后建立 | 仅预留目录，不是 ROS 包 |
| 自定义接口 | `src/d_task_interfaces` | 仅在已有接口分支确认后保留 | 当前不存在，不创建 |
| PX4 消息 | `src/px4_msgs` | `dependencies/px4_msgs.repos` | 外部依赖，不提交 |

## 版本冻结

- 飞控：Holybro Pixhawk 6C Mini
- PX4：Release v1.16.0
- PX4 提交：`6ea3539157ca358c70a515878b77077af7d4611d`
- PX4 编译目标：`px4_fmu-v6c_default`
- 操作系统：Ubuntu 24.04
- ROS：ROS 2 Jazzy
- `px4_msgs`：`release/1.16`
- `px4_msgs` 提交：`392e831c1f659429ca83902e66820d7094591410`
- Micro XRCE-DDS Agent：v2.4.3

任何模块合并前都必须证明与上述版本兼容。基线检查脚本只报告问题，不自动
切换分支、下载依赖或改写版本。

## 合并顺序

1. 锁定本集成骨架及外部依赖版本。
2. 收集各负责人提供的功能分支、完整提交哈希、测试结果和接口说明。
3. 合并已确认的飞控任务控制分支并进行无桨、无解锁回归。
4. 合并已确认的小车与联合仿真提交，保留原坐标和测试基线。
5. 合并已确认的离线只读地面站提交。
6. 审计跨模块话题、服务、消息和配置后建立真实 `d_task_bringup` 包。
7. 依次完成隔离构建、软件在环、硬件在环和现场人工验收。

未经负责人给出明确分支和提交，不得用工作树脏改动或目录复制代替合并。

## 编译与部署边界

`colcon` 只扫描 `src/` 下含 `package.xml` 的 ROS 包。当前骨架中的
`ground_vehicle`、顶层 `ground_station`、`openmv_h7plus`、`dependencies`、
`config`、`docs` 和 `scripts` 不参与 `colcon` 编译。

- `openmv_h7plus` 中四个正式 Python 文件烧录到 OpenMV H7 Plus。
- 小车 MCU 固件与板级通信代码以后放入 `ground_vehicle`，由对应工具链烧录；
  ROS 小车节点仍由树莓派工作区编译。
- `src/uav_vision`、`src/uav_control`、`src/ground_station` 及以后确认的 ROS
  包在树莓派上由 `colcon` 编译。
- 顶层 `ground_station` 保存离线地面站的非 ROS 部署资源；不得向飞控发布
  控制指令。
- `src/px4_msgs` 由依赖清单导入，只作为外部 ROS 接口依赖，不提交到仓库。
