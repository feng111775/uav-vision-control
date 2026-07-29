# PX4 v1.16.0 SITL 实测报告

日期：2026-07-29（Asia/Shanghai）。

## 版本与启动依据

此前只搜索目录名严格等于 `PX4-Autopilot`，因而漏掉了有效仓库。实际源码位于
`/home/xixi/PX4-Autopilot-1.16.0`：

- HEAD：`6ea3539157ca358c70a515878b77077af7d4611d`
- tag：`v1.16.0`
- Git 状态：detached HEAD，工作树干净；本次没有修改 PX4 源码。
- `ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500` 和 `make help`
  确认标准模型为 `gz_x500`。
- 实际启动：在上述目录执行 `make px4_sitl gz_x500`。
- Gazebo Sim：8.11.0，实际 world `default`，model `x500_0`。

PX4 `rcS` 与启动日志均显示 uXRCE-DDS 使用 UDP `127.0.0.1:8888`。Agent 为
`/usr/local/bin/MicroXRCEAgent`，动态库 SONAME 为
`libmicroxrcedds_agent.so.2.4`（固定安装 v2.4.3），实际命令：

```bash
MicroXRCEAgent udp4 -p 8888 -v 4
```

ROS 为 Jazzy；`px4_msgs` 为 `release/1.16` 的冻结提交
`392e831c1f659429ca83902e66820d7094591410`。

## DDS 实际接口

实际 `/fmu` 清单包含控制器所需的：

- `/fmu/out/vehicle_status_v1`
- `/fmu/out/vehicle_local_position`
- `/fmu/out/vehicle_attitude`
- `/fmu/out/vehicle_command_ack`
- `/fmu/in/offboard_control_mode`
- `/fmu/in/trajectory_setpoint`
- `/fmu/in/vehicle_command`

重要实测差异：PX4 v1.16 的 `VehicleLocalPosition.MESSAGE_VERSION=0`，因此运行时
话题没有 `_v1` 后缀；`VehicleStatus.MESSAGE_VERSION=1` 才有 `_v1`。控制器和
SITL 场景已改为真实的 `/fmu/out/vehicle_local_position`，没有修改消息定义。

频率（约 6 秒窗口）：`vehicle_status_v1` 稳态约 2 Hz，
`vehicle_local_position` 约 100.0 Hz，`vehicle_attitude` 约 100.0 Hz。
只读配置到达 `WAIT_START` 并记录 H 点；对三个 PX4 输入话题各监听 1 秒均为
0 条控制消息，未解锁；mission dashboard 约 10 Hz。

## 闭环结果

| 测试 | 最终结果 | 实测摘要 |
|---|---|---|
| 原生 x500 | 通过 | PX4 v1.16.0、default world、x500_0 正常启动 |
| 只读通信 | 通过 | H 点/姿态/位置正常，无 PX4 控制输出 |
| hover_test | `COMPLETE` | 总计约 28.04 s；目标 1.0 m，最大高度约 1.03 m；悬停不少于 10 s |
| drop | `COMPLETE` | mission telemetry 计时 32.10 s；release 1 次、D 前 ack、返回 H 并自动上锁 |
| dynamic_land | `COMPLETE` | mission telemetry 计时 59.55 s；两次起飞/降落和二次返回闭环 |

hover_test CommandAck：DO_SET_MODE(176)、ARM(400)、NAV_LAND(21) 均为
`ACCEPTED(0)`。H 点 z 约 -0.04 m，悬停 z 最低约 -1.032 m，实际高度约
0.99--1.03 m。

drop 最终 telemetry 时间线：

- HOVER_CONFIRM：14.50--14.65 s；此前 TAKEOFF 已连续稳定。
- FOLLOW_TARGET：14.75--20.75 s。
- DROP_RELEASE：20.85 s，小车进度为 `PASSED_B`。
- release：仅 1 次；mock ack 收到后立即返回。
- RETURN_H：20.90--23.60 s，进入 LAND_H 前 H 水平误差约 0.083 m。
- LAND_H：23.65--29.60 s；COMPLETE：32.10 s。
- 场景 B 点为 10 s、C 点为 25 s、D 点为 45 s，因此 release 明确早于 D。
- 有效视觉帧归一化误差模长平均约 0.376、最大约 0.550；release 帧约 0.189。

dynamic_land 最终 telemetry 时间线：

- TAKEOFF 11.30--11.65 s，HOVER_CONFIRM 11.70--14.70 s。
- FOLLOW_TARGET 14.80--19.95 s，DESCEND_ON_CAR 20.05--26.95 s。
- TOUCHDOWN_VERIFY 27.00--29.00 s，DISARM_ON_CAR 29.05--32.45 s。
- DWELL_ON_CAR 32.50--37.45 s；20 Hz 边界采样差为一帧，状态机门控为不少于 5.0 s。
- SECOND_PRESTREAM/ARM 后 SECOND_TAKEOFF 38.60--48.95 s。
- RETURN_H 49.00--51.65 s，进入 LAND_H 前 H 水平误差约 0.083 m。
- LAND_H 51.70--57.45 s，COMPLETE 59.55 s。
- 第二次起飞使用记录的平台落点 x/y 垂直上升至 1.5 m，再返回初始 H。

普通 x500 没有移动平台模型。首次下降验证了真实 PX4 水平闭环、下降门控、
近地慢降和地面落地动力学；`sitl_nav_land_after_touchdown=true` 仅在
`sitl_dynamic_land.yaml` 中启用，让 PX4 原生 NAV_LAND/落地检测/自动上锁
完成 mock 接触后的逻辑衔接。competition 配置不启用该适配。因此上述结果不能
证明移动平台接触、摩擦、承载或平台上二次起飞稳定性。

## 故障注入

| 注入 | 节点级实测结果 |
|---|---|
| 短暂视觉丢失 0.6 s | dynamic_land 在下降链路中停止/恢复，最终 `COMPLETE` |
| 持续视觉丢失 | 从 `DESCEND_ON_CAR` 转安全返回，随后 `LAND_H → WAIT_DISARM → COMPLETE`，未盲降 |
| 提前通过 D | drop 未进入 release，转返回 H 并 `COMPLETE` |
| payload ack 丢失 | release 状态超时后安全返回并 `COMPLETE`；任务不记为抛投成功，不无限发布 |
| PX4 退出 Offboard | 真实进入 `EXTERNAL_CONTROL`，未重新请求 Offboard |
| Agent 中断 | 首次实测发现运行中失鲜未进入安全状态；修复后 Agent `SIGTERM` 约 4 s 内进入 `FAILSAFE_LAND` |
| 任务超时 | SITL 专用 8 s 覆盖参数触发 `FAILSAFE_LAND`；competition 仍为 90 s |

## 实测发现与修复

1. 本地位置真实 topic 无 `_v1`；集中定义并修复控制器与场景订阅。
2. PX4 输出 QoS 为 best-effort/transient-local；修复场景订阅兼容性。
3. 场景图像误差必须按机头航向转换；修复相机右方向符号和 NED 映射。
4. D 点保护补齐 SEARCH/FOLLOW；抛投高度门控不再错误要求伴飞水平速度为零。
5. NAV_LAND 后停止巡航 setpoint，避免与 PX4 自动降落冲突。
6. 动态触地接近阈值改为 YAML；SITL 以 H 地面高度的 3 cm 为保守阈值。
7. 平台主动上锁阶段允许预期 Offboard 退出，不误判人工接管。
8. 二次起飞记录落点，先垂直升高；并重置 mode/arm/land/disarm 全部命令跟踪器。
9. 运行中 PX4 状态或位置失鲜现在进入 `FAILSAFE_LAND`。

所有原始日志保存在 `/tmp/d-task-sitl-logs/`，不纳入 Git。

## 最终回归

- `python3 -m compileall src/uav_control src/uav_vision`：通过。
- 新隔离目录：`/tmp/d-task-sitl-final.e7aSjs`。
- `uav_vision`、`uav_control` 构建：2 包成功。
- 测试：162 项，0 error，0 failure，1 skip。
- skip 为 ROS 包模板既有的 copyright-header 检查（生成源码没有版权头），与功能
  和本次修改无关；原 149 项没有倒退，新增回归均通过。
