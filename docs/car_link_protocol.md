# 小车只读链路协议

树莓派只接收状态，不发布速度、电机、转向或路线命令。串口与 UDP 使用相同
ASCII 帧：

`CAR,1,sequence,timestamp_ms,progress,flags,crc16`

CRC 是前六字段（含逗号）的 CRC16-CCITT（初值 `0xffff`），以四位十六进制
表示。`sequence` 为 uint16，按半范围规则判断新旧并允许 `65535→0` 回绕；
重复、乱序、CRC/版本错误以及 progress 倒退均丢弃。progress 固定为 0～5，
flags bit0 表示任务从 A 点启动。

`disabled` 不打开接口；`udp` 仅绑定并接收，必须显式配置地址和端口，可限制
远端 IP；`serial` 只读且必须显式配置稳定设备别名和波特率；`replay` 只用于
测试。模拟器只允许 bench/test，正式 competition launch 不启动它。
