# QR shelf 最终验证摘要

本目录只放精选、可复现证据。训练集 160 图、验证 48、测试 48；正式 SVM
验证 patch precision=0.97297、recall=0.96429、accuracy=0.97266，单 patch
CPU 分类约 0.002 ms（不含滑窗和解码）。

首轮 held-out 24 图/50 框端到端结果：

| 后端 | 定位成功率 | 内容解码率 | 平均整图延迟 |
|---|---:|---:|---:|
| OpenCV | 20% | 20% | 12.33 ms |
| 模型候选 + OpenCV | 36% | 36% | 1177.77 ms |

这组强增强数据揭示滑窗 SVM 虽提高成功率但不适合作为实时默认；正式默认
仍建议 `hybrid`（存在权重则模型辅助，否则 OpenCV），后续树莓派/Coral 应
替换为 nano ONNX/TFLite。没有把 patch precision 冒充 mAP；当前 SVM 不输出
标准目标检测 mAP50/mAP50-95，因此记为 N/A。

已验证：生成、实际训练、核心单测、ROS 构建/测试、静态 launch 安全。
`colcon test-result --verbose` 为 145 tests、0 errors、0 failures、1 skipped
（即 144 passed）；干净 QR 7 离线 smoke 在 opencv 与 hybrid 均正确输出 7。
未验证：需要 Gazebo/QGC 图形交互的完整起飞—瞄准—降落，以及真机/Coral。
这些限制不写成成功结果。
