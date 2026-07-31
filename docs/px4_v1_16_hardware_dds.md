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

已验证的真机链路为 Pixhawk 6C Mini 的 GPS2（`/dev/ttyS6`）到树莓派
`/dev/ttyAMA0`，两端均使用 921600。PX4 端使用 `UXRCE_DDS_CFG=202`
选择 GPS2，`GPS_2_CONFIG=0` 保持 GPS2 串口为普通串口。PX4 端与树莓派
Agent 的波特率必须完全一致；本仓库不会自动修改 PX4 参数，参数仍需在
真机上只读核验。

树莓派正式 Agent 启动命令为：

```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600 -v 4
```

PX4 手动排错命令为：

```text
uxrce_dds_client stop
uxrce_dds_client start -t serial -d /dev/ttyS6 -b 921600
```

先完成人工身份、线序、电平和参数核对，再可用包装器进行 dry-run：

```bash
scripts/pi/start_microxrce_agent.sh --transport serial \
  --device /dev/ttyAMA0 --baudrate 921600 --dry-run
```

包装器缺少任一值即失败，不使用 sudo。真机必须重新核对七个冻结 `/fmu`
话题，尤其本项目实际 local-position 名称为
`/fmu/out/vehicle_local_position`。
