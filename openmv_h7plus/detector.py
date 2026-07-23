"""基于 OpenMV find_blobs 的红色色块检测。"""

from thresholds import RED_THRESHOLDS


PIXELS_THRESHOLD = 120
AREA_THRESHOLD = 180


def _clamp(value, lower, upper):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def _engineering_confidence(blob, frame_area):
    """根据填充率和目标尺寸生成 0~100 的工程评分。"""
    # OpenMV 5.0 的 Blob 是属性元组，几何量和像素数不是可调用方法。
    box_area = blob.w * blob.h
    if box_area <= 0 or frame_area <= 0:
        return 0

    fill_score = (blob.pixels * 100) // box_area
    # 占画面 5% 时尺寸项达到满分，避免大噪点轻易得到高分。
    size_score = (blob.pixels * 2000) // frame_area
    size_score = _clamp(size_score, 0, 100)
    confidence = (fill_score * 7 + size_score * 3) // 10
    return int(_clamp(confidence, 0, 100))


def detect_red(image):
    """返回最大红色色块的协议字段字典；无目标时返回 None。"""
    blobs = image.find_blobs(
        RED_THRESHOLDS,
        pixels_threshold=PIXELS_THRESHOLD,
        area_threshold=AREA_THRESHOLD,
        merge=True,
    )
    if not blobs:
        return None

    # pixels 是通过颜色阈值的实际像素数，用它选择目标并作为 area 输出。
    target = max(blobs, key=lambda blob: blob.pixels)
    frame_area = image.width() * image.height()
    return {
        "valid": 1,
        "cx": target.cx,
        "cy": target.cy,
        "width": target.w,
        "height": target.h,
        "area": target.pixels,
        "confidence": _engineering_confidence(target, frame_area),
    }
