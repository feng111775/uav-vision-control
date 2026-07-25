# PX4 SITL 双摄闭环验证

最终验证环境：

- Ubuntu 24.04、ROS 2 Jazzy
- PX4 v1.17.0 (`d6f12ad1c4`)
- Gazebo Sim 8.11.0
- 世界 `red_target`
- 机型 `gz_x500_downward_camera`
- 正式控制节点 `vision_offboard_controller`

最终一次飞行完成：

```text
WAITING → PRESTREAM → TAKEOFF → VISION_CONTROL
→ SEARCH/front → DOWN_ACQUIRE/front → ALIGN/down
→ LAND → DISARM
```

| 指标 | 实测值 |
| --- | ---: |
| 最大高度 | 2.023 m |
| front→down 切换位置（NED） | (-0.112, 3.179, -1.989) m |
| 切换时前视面积 | 34205.5 px² |
| 下视初始中心误差 | 117.3 px |
| 下视最终中心误差 | 3.5 px |
| 最终对准位置（NED） | (-0.011, 4.765, -1.995) m |
| PX4 failsafe | 未发生 |
| 结束状态 | Landing detected / Disarmed by landing |

验证图：

- `final_summary.png`：Gazebo、QGC、双相机窗口和轨迹汇总
- `final_xy_trajectory.png`：实际 XY 轨迹和阶段标记
- `final_height.png`：高度曲线
- `final_visual_errors.png`：前视目标增长与下视误差收敛

完整 rosbag 和 ULog 体积较大，保存在验证机器的
`test_results/dual_camera_final/`，不纳入 Git。
