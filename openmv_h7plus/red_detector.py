"""Experimental LAB red-centre helper for the D-task landing target.

This module is intentionally independent from ``main.py`` and the formal
black-ring/cross detector.  It accepts an OpenMV RGB565 ``image`` object and
returns only an auxiliary red-sticker observation; it never serializes a
``D_TARGET`` frame or claims that the complete target was detected.
"""

try:
    import math
except ImportError:  # Kept for MicroPython compatibility.
    math = None


# OpenMV LAB tuple layout is (L min, L max, a min, a max, b min, b max).
# The overlapping bands intentionally tolerate normal red hue changes caused
# by illumination and specular highlights.  They are a starting point for
# hardware calibration, not formal-target acceptance thresholds.
RED_LAB_THRESHOLDS = (
    (20, 100, 25, 127, 0, 127),
    (20, 100, 15, 127, -20, 127),
)

MIN_RED_PIXELS = 40
MIN_RED_AREA = 80
MIN_RED_DIAMETER_PX = 8
MAX_RED_DIAMETER_PX = 180
MIN_FILL_RATIO = 0.35
MIN_ASPECT_RATIO = 0.60
MIN_CONFIDENCE = 45


def _value(item, name, index, default=0):
    """Read an OpenMV result method or a tuple-like test double."""
    value = getattr(item, name, None)
    if value is None:
        try:
            value = item[index]
        except (IndexError, KeyError, TypeError):
            return default
    return value() if callable(value) else value


def _clamp(value, lower=0.0, upper=1.0):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def _invalid():
    return {"valid": 0, "cx": 0, "cy": 0, "area": 0, "confidence": 0}


def _candidate(blob, image_width, image_height):
    x = int(_value(blob, "x", 0))
    y = int(_value(blob, "y", 1))
    width = int(_value(blob, "w", 2))
    height = int(_value(blob, "h", 3))
    pixels = int(_value(blob, "pixels", 5))
    area = max(1, int(_value(blob, "area", 6, width * height)))
    diameter = max(width, height)
    if (width <= 0 or height <= 0 or pixels < MIN_RED_PIXELS or
            area < MIN_RED_AREA or diameter < MIN_RED_DIAMETER_PX or
            diameter > MAX_RED_DIAMETER_PX):
        return None

    aspect = min(width, height) / max(1.0, max(width, height))
    fill_ratio = pixels / float(area)
    if aspect < MIN_ASPECT_RATIO or fill_ratio < MIN_FILL_RATIO:
        return None

    # A filled circular sticker has near-unity aspect and density.  A small,
    # bounded image-centre term resolves equal red background candidates but
    # never rejects an off-centre target.
    circular_score = _clamp((aspect - MIN_ASPECT_RATIO) /
                            (1.0 - MIN_ASPECT_RATIO))
    fill_score = _clamp((fill_ratio - MIN_FILL_RATIO) /
                        (1.0 - MIN_FILL_RATIO))
    area_score = _clamp(pixels / float(max(MIN_RED_PIXELS * 8,
                                           image_width * image_height // 16)))
    cx = x + width // 2
    cy = y + height // 2
    dx = cx - image_width / 2.0
    dy = cy - image_height / 2.0
    distance = (dx * dx + dy * dy) ** 0.5
    diagonal = max(1.0, (image_width * image_width +
                         image_height * image_height) ** 0.5)
    centre_score = _clamp(1.0 - distance / diagonal)
    confidence = int(100 * (0.34 * circular_score + 0.34 * fill_score +
                            0.24 * area_score + 0.08 * centre_score))
    return {
        "valid": int(confidence >= MIN_CONFIDENCE),
        "cx": cx,
        "cy": cy,
        "area": area,
        "confidence": max(0, min(100, confidence)),
    }


def detect_red_center(image):
    """Return the best LAB red-sticker observation from an OpenMV image.

    ``image`` must be an RGB565 OpenMV image because the formal grayscale
    camera setting contains no chroma information.  No exception escapes for
    an absent or malformed image; callers receive an invalid observation.
    """
    try:
        image_width = int(image.width())
        image_height = int(image.height())
        if image_width <= 0 or image_height <= 0:
            return _invalid()
        blobs = image.find_blobs(
            RED_LAB_THRESHOLDS, pixels_threshold=MIN_RED_PIXELS,
            area_threshold=MIN_RED_AREA, merge=True, margin=2)
    except (AttributeError, TypeError, ValueError, OSError):
        return _invalid()

    best = None
    for blob in blobs or ():
        candidate = _candidate(blob, image_width, image_height)
        if candidate is None:
            continue
        if best is None or candidate["confidence"] > best["confidence"]:
            best = candidate
    return best if best is not None else _invalid()
