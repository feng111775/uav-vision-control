# PX4 v1.16 真机 DDS 传输边界

依据只读检查的
`/home/xixi/PX4-Autopilot-1.16.0/src/modules/uxrce_dds_client`（tag
v1.16.0，hash `6ea3539157ca358c70a515878b77077af7d4611d`），v1.16 客户端
支持 `serial` 与 `udp`，参数/入口包含 `UXRCE_DDS_CFG`、
`UXRCE_DDS_DOM_ID`、`UXRCE_DDS_KEY`、`UXRCE_DDS_PRT` 和
`UXRCE_DDS_AG_IP`。源码帮助示例展示 `-t serial -d ... -b ...`；这只是
能力证明，不是本机正式端口或波特率。

SITL 已验证的 Agent 形式是：

```bash
MicroXRCEAgent udp4 -p 8888
```

真机预计采用经电气和线序确认的 TELEM USB-UART 串口，或经过验证的 USB
链路。实际端口、稳定设备别名、波特率和 PX4 参数均为未配置。本仓库不会
自动修改 PX4 参数。先完成人工身份、线序、电平和参数核对，再用：

```bash
scripts/pi/start_microxrce_agent.sh --transport serial \
  --device /dev/dtask_pixhawk --baudrate <实测值> --dry-run
```

包装器缺少任一值即失败，不使用 sudo。真机必须重新核对七个冻结 `/fmu`
话题，尤其本项目实际 local-position 名称为
`/fmu/out/vehicle_local_position`。
