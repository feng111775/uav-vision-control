# QR shelf datasets

完整数据集由 `src/uav_vision/tools/generate_qr_dataset.py` 生成并被
`.gitignore` 排除。固定种子为 `20260726`，train/val/test 使用不相交的
变体族；配置与统计写入数据集目录。`samples/` 仅保留可提交的解码样例。

```bash
python3 src/uav_vision/tools/generate_qr_dataset.py \
  --output datasets/qr_shelf --train 160 --val 48 --test 48
```
