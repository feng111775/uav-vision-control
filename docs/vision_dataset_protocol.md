# D题视觉数据集与标注协议

原图默认位于未跟踪的`datasets/d_task/`，不得直接git add。建议结构：

```text
datasets/d_task/
├── images/
│   ├── positive/
│   └── negative/
└── annotations.csv
```

采集必须包含30/50 cm真实打印目标、无目标负样本，以及不同高度、画面位置、旋转、
均匀/不均匀光照、阴影、高光、运动模糊、部分遮挡、边界截断和相似圆环干扰。
保留原图，不反复JPEG转码。

创建标注模板：

```bash
PYTHONPATH=src/uav_vision python3 \
  src/uav_vision/tools/annotate_d_task_dataset.py \
  datasets/d_task datasets/d_task/annotations.csv
```

CSV字段：

```text
path,has_target,center_x,center_y,outer_diameter,inner_diameter,angle_rad,lighting,height_m,notes
```

角度规范化到`[-π/4, π/4)`，按π/2周期比较。无目标行只填写path、has_target=0和
光照分类。至少两人复核边界、遮挡和角度困难样本。

评估：

```bash
PYTHONPATH=src/uav_vision python3 \
  src/uav_vision/tools/evaluate_d_task_dataset.py datasets/d_task \
  --annotations datasets/d_task/annotations.csv \
  --debug-dir /tmp/d-task-debug --output /tmp/d-task-report.json
```

没有标注时只能报告候选数量和处理耗时，不能称为真实准确率。

ROS现场记录：

```bash
ros2 bag record /vision/h7/detection /vision/h7/status \
  /vision/target/tracked /vision/landing_error /vision/debug/status
ros2 topic hz /vision/h7/detection
ros2 topic hz /vision/target/tracked
ros2 topic hz /vision/landing_error
```
