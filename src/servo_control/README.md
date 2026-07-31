# servo_control

独立的 ROS 2 Jazzy 舵机投放节点。默认是 dry-run，不会连接 pigpio，也不会驱动 GPIO；本包当前不订阅视觉话题，不修改 `uav_control` 主任务状态机。

## 接线与安全

- 树莓派 BCM GPIO18 → 舵机信号线。
- 外部 5V/BEC 正极 → 舵机红线。
- 外部 5V/BEC 负极 → 舵机 GND。
- 树莓派 GND → 外部 5V/BEC 负极。

必须共地。不建议用树莓派 5V 引脚给负载较大的舵机供电。第一次测试应拆掉投放物和螺旋桨；先 dry-run，再接信号线空载测试。角度和脉宽必须小步调整，避免舵机堵转。

## 依赖与 pigpiod

在 Ubuntu 24.04 / Raspberry Pi 设备上，先检查软件包和服务名称，不把它们视为本开发机已验证事实：

```bash
apt-cache policy python3-pigpio
dpkg -L python3-pigpio 2>/dev/null | head
systemctl list-unit-files | grep -i pigpio
command -v pigpiod
```

若软件源提供 `python3-pigpio`，再按设备实际结果安装。根据检查到的服务名启动；若没有 systemd 单元，可直接检查并启动 `pigpiod`：

```bash
sudo apt update
sudo apt install python3-pigpio
sudo systemctl start pigpiod.service   # 仅当上面的检查确认该服务名存在
pigpiod -v
```

节点用 `pigpio.pi()` 检查连接；连接失败时不会进入 `spin`。本次未在树莓派上验证 pigpio 包、服务名或连接。

## 构建与启动

```bash
cd /home/a-corn/px4_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select servo_control --symlink-install
source /home/a-corn/px4_ros2_ws/install/setup.bash
```

默认 dry-run：

```bash
ros2 launch servo_control servo_control.launch.py
```

真机必须显式选择 real 配置，且只能在确认树莓派、接线和机构安全后执行：

```bash
ros2 launch servo_control servo_control.launch.py config:=$(ros2 pkg prefix servo_control)/share/servo_control/config/servo_real.yaml
```

## 话题操作

```bash
ros2 topic pub --once /servo_command std_msgs/msg/String "{data: prepare}"
ros2 topic pub --once /servo_command std_msgs/msg/String "{data: throw}"
ros2 topic pub --once /servo_command std_msgs/msg/String "{data: reset}"
ros2 topic pub --once /servo_command std_msgs/msg/String "{data: stop}"
ros2 topic pub --once /servo_command std_msgs/msg/String "{data: unknown}"
ros2 topic echo /servo/status --qos-durability transient_local
ros2 topic echo /servo/result
```

status 使用 reliable + transient-local，新订阅者可收到最近状态；查看 status 时要保留 `--qos-durability transient_local`。command/result 使用 reliable QoS。

安全停止：发送 `stop`，然后用 `Ctrl-C` 退出节点。退出清理会取消定时器、尝试发送 0 脉宽并关闭 pigpio；清理异常只记录日志，不阻塞退出。

## 参数

| 参数 | 默认值 |
| --- | --- |
| `gpio_pin` | `18` |
| `safe_angle_deg` | `0.0` |
| `release_angle_deg` | `90.0` |
| `min_angle_deg` / `max_angle_deg` | `0.0` / `180.0` |
| `min_pulse_us` / `max_pulse_us` | `1000` / `2000` |
| `prepare_duration_sec` | `0.5` |
| `release_duration_sec` | `0.8` |
| `return_duration_sec` | `0.5` |
| `return_after_release` | `true` |
| `disable_pwm_after_action` | `true` |
| `allow_repeat` | `false` |
| `dry_run` | `true` |
| `command_topic` | `/servo_command` |
| `status_topic` | `/servo/status` |
| `result_topic` | `/servo/result` |

1000–2000 us 是保守示例，不适合所有舵机；必须依据具体舵机数据手册和空载标定调整。`safe_angle_deg` 是上电后的安全准备角度，节点不会在启动时自动转动舵机。

## 标定

先 dry-run，再真机空载；从安全角度和窄脉宽范围开始，每次只改小步，确认机构两端不会卡死。记录“刚开始运动、目标位置、机械限位”对应的脉宽，设置 `min_pulse_us`、`max_pulse_us` 和两个动作角度。空载动作通过不代表挂载荷投放通过。

## 接口与状态

输入 `/servo_command` (`std_msgs/msg/String`) 支持 `prepare`、`throw`、`reset`、`stop`。输出 `/servo/status` 和 `/servo/result` 均为 `std_msgs/msg/String`。`SUCCESS:throw` 只表示舵机动作序列完整执行，不是视觉对准成功或正式主任务 ACK。`throw` 默认只能完成一次；忙碌或已完成时拒绝，`reset` 不清除完成锁。
