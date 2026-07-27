"""Shared validation and normalized geometry for camera detections."""

import math


VALUE_COUNT = 9
INVALID_DETECTION = [0.0] * VALUE_COUNT


def validate_detection(values):
    """Return one finite nine-field detection with valid image geometry."""
    if len(values) != VALUE_COUNT:
        raise ValueError(
            'detection must contain exactly 9 values, got %d' % len(values))
    data = [float(value) for value in values]
    if not all(math.isfinite(value) for value in data):
        raise ValueError('detection values must be finite')
    if data[0] not in (0.0, 1.0):
        raise ValueError('valid must be 0 or 1')
    if any(value < 0.0 for value in data[1:6]):
        raise ValueError('detection geometry cannot be negative')
    if not 0.0 <= data[6] <= 100.0:
        raise ValueError('confidence must be in [0, 100]')
    if data[0] == 0.0 and data[7] == 0.0 and data[8] == 0.0:
        return data
    if data[7] <= 0.0 or data[8] <= 0.0:
        raise ValueError('image width and height must be positive')
    if data[1] > data[7] or data[2] > data[8]:
        raise ValueError('target center must be inside the image')
    if data[3] > data[7] or data[4] > data[8]:
        raise ValueError('target size must not exceed the image')
    if data[5] > data[7] * data[8]:
        raise ValueError('target area must not exceed the image area')
    return data


def normalized_geometry(values):
    """Return error_x, error_y, area, width and height ratios."""
    data = validate_detection(values)
    image_width = data[7]
    image_height = data[8]
    return (
        (data[1] - image_width / 2.0) / (image_width / 2.0),
        (data[2] - image_height / 2.0) / (image_height / 2.0),
        data[5] / (image_width * image_height),
        data[3] / image_width,
        data[4] / image_height,
    )
