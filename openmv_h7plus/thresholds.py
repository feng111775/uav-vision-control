"""比赛现场需要频繁调整的颜色阈值。"""

# OpenMV LAB 阈值顺序：(L_min, L_max, A_min, A_max, B_min, B_max)。
# 下面只是一组红色目标的起始值，必须根据现场照明和材料重新采样。
RED_THRESHOLDS = [
    (25, 100, 20, 127, 10, 127),
]

