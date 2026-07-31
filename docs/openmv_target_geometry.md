# OpenMV 标定目标几何

该文件仅用于 **OpenMV 算法标定与真机验收**，**不是**比赛正式目标定义。若比赛最终尺寸另有发布，以比赛规则为准。

## 当前算法要求

- 外圆直径允许范围：`24 px` 到 `220 px`
- 内外圆直径比例：`0.52` 到 `0.68`，名义比例 `0.60`
- 同心误差：不超过外圆直径的 `10%`
- 十字长度：建议总长度约为内圆直径的 `0.67`，当前标定目标为 `64 mm`
- 十字线宽：建议与圆环线宽一致，当前标定目标为 `8 mm`
- 十字位置：必须位于内圆内，并经过圆心
- 黑白极性：白底黑环、白底黑十字
- 推荐打印尺寸：外圆 `160 mm`，内圆 `96 mm`，十字总长 `64 mm`，线宽 `8 mm`
- 1.5 m 处建议像素直径：约 `28–30 px`（按 QVGA `320x240`、OV5640 常见水平视场约 `62°` 估算）
- 最小可识别像素尺寸：外圆直径至少 `24 px`，更稳妥建议不低于 `28 px`

## 资产文件

- 矢量打印版：`docs/assets/openmv_test_target.svg`
- 位图打印版：`docs/assets/openmv_test_target.png`
- 生成脚本：`tools/generate_openmv_test_target.py`

## 与算法参数的一致性

- 标定目标比例固定为 `96 / 160 = 0.60`
- 该比例落在检测器接受区间 `0.52–0.68` 中心附近
- 十字完全位于内圆内，避免把十字端点推出圆外导致 Hough 线段中心约束失配

## 一次性调试图像

默认 `DEBUG_CAPTURE_ONCE=False`。仅在需要复盘阈值分割时临时打开：

1. 编辑 `openmv_h7plus/config.py`，将 `DEBUG_CAPTURE_ONCE` 设为 `True`
2. 重启 OpenMV，等待生成一次 `debug_raw.pgm` 和 `debug_threshold.pgm`
3. 从 OpenMV 磁盘根目录取出 `/flash/debug_raw.pgm` 与 `/flash/debug_threshold.pgm`
4. 立即把 `DEBUG_CAPTURE_ONCE` 改回 `False` 并重新部署

该功能只写入一次；保存失败不会中断视觉循环。

## 任务模式说明

截至 `2026-07-31`，OpenMV 侧默认以 `SEARCH` 模式运行。当前仓库尚未在 `openmv_h7plus/main.py` 中接入稳定的 Pi→OpenMV 任务模式命令链，因此 **不会伪造 `FOLLOW` 或 `DROP_ALIGN` 输入**；这两个模式仅保留在检测器接口与协议枚举中，供后续链路打通后启用。
