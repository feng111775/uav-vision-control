# d_task_bringup

第一题统一正式入口位于
`launch/competition_first_task.launch.py`。必须显式选择 profile：

- `readonly_bench`
- `no_prop_control`
- `real_competition`

真实 profile 由启动前检查 fail-closed；相机安装验收标记、DDS/PX4、UDP 和真实
舵机条件未满足时不会启动真实任务链路。
