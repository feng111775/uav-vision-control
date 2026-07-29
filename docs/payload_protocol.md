# 抛投机构桥协议

任务控制器仍只使用 `/uav/payload/release` 与
`/uav/payload/release_ack`。桥节点不决定抛投时机。

- 主机：`DROP,1,sequence,crc16`
- 执行器：`DROP_ACK,1,sequence,result,crc16`

result 为 SUCCESS(0)、BUSY(1)、MECHANICAL_ERROR(2)、SENSOR_ERROR(3)、
DENIED(4)、UNKNOWN_ERROR(5)。只有相同 sequence 的 SUCCESS 才发布一次
ROS true ack。持续 true 只产生一个动作；false 重新武装边沿检测。超时有限
重试，旧 ack、错误 ack 和超时都不会伪造成功。重启后不恢复未确认动作。

支持 `disabled`、`mock`、`serial`；GPIO/PX4 PWM 未实现并会明确拒绝。
mock 只用于 bench，competition 配置验证器会拒绝它。
