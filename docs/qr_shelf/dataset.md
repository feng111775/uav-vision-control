# 数据集

生成器固定种子 `20260726`，生成 640×480 白色多层货架、1–4 个目标和
15% 负样本，包含透视、小目标、边缘截断、遮挡、阴影、明暗、运动/高斯
模糊、噪声、JPEG 失真及黑白干扰。二维码文本为十进制 1–24，物理配置
0.19 m，OpenCV 编码器的 ECC=M、quiet zone=4。每个 split 使用独立 family
编号，避免同族泄漏。

```bash
python3 src/uav_vision/tools/generate_qr_dataset.py \
  --output datasets/qr_shelf --train 160 --val 48 --test 48
```

本次实际规模 256 图：train 160/375 框，val 48/112 框，test 48/105 框。
完整目录不提交；`statistics.json` 会随重建生成。
