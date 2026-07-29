# D题第三阶段A视觉测试报告

日期：2026-07-29。

## 结果分层

| 层级 | 当前结果 |
|---|---|
| 单元测试 | 两包共202项，0 error、0 failure、1个既有copyright模板skip |
| 合成图 | 24张回放，22张产生候选；平均1.01 ms、P95 1.35 ms；仅方向回归 |
| 未标注回放 | 现有256张QR shelf回放为0个候选，平均2.09 ms、P95 3.73 ms；不是D题负样本标注，不能称误检率 |
| 已标注实拍 | 不存在，不能计算真实检出率/误检率/像素误差 |
| OpenMV真机 | 未连接H7 Plus，未部署、无FPS |
| ROS串口链 | fake正式链路clean启动；H7未连接，不把fake频率当H7频率 |

PC参考检测器不是比赛实时节点。合成图通过不代表30/50 cm打印目标在真实光照、
运动模糊或相机畸变下的准确率。

## 接口

`/vision/h7/detection`七字段顺序、tracked和landing_error均未改变。新增的
`/vision/h7/status`是只读字符串诊断，仅供dashboard区分TRACKING、LOST、
CROSS_INVALID和DETECT_ERROR；任务控制不订阅。

## 禁止生成的结论

当前没有生成相机矩阵、畸变参数、焦距、米制落点误差、平台触地高度、真实检出率、
OpenMV FPS或H7串口频率。

## ROS fake烟雾

- 节点仅有fake_h7、filter、predictor、landing_error、dashboard；没有legacy和PX4控制器。
- `/vision/h7/detection`：10.000 Hz。
- `/vision/target/tracked`：10.000 Hz。
- `/vision/landing_error`：约43.6 Hz（既有回调/定时发布行为）。
- `/vision/debug/target_canvas`：10.000 Hz。
- `/vision/debug/status`：`valid=1 state=TRACKING`并包含中心、直径、角度、confidence、
  age和预测中心。
- 当前DDS图中有历史mission participant的幽灵PX4 endpoint，但进程表和
  `ros2 node info`均不存在该节点；本launch源码及实际node list不含任何PX4发布节点。
