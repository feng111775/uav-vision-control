"""Canonical array schemas and validation for the 2026 D-task vision chain."""

import math


DETECTION_LENGTH = 7
TRACKED_LENGTH = 12
LANDING_ERROR_LENGTH = 8

VALID = 0
CENTER_X = 1
CENTER_Y = 2
OUTER_DIAMETER = 3
INNER_DIAMETER = 4
ANGLE = 5
CONFIDENCE = 6

VELOCITY_X = 7
VELOCITY_Y = 8
PREDICTED_CENTER_X = 9
PREDICTED_CENTER_Y = 10
TARGET_AGE_MS = 11

ERROR_X_NORMALIZED = 1
ERROR_Y_NORMALIZED = 2
ERROR_X_PX = 3
ERROR_Y_PX = 4
ANGLE_ERROR = 5
ERROR_CONFIDENCE = 6
ERROR_TARGET_AGE_MS = 7

ANGLE_PERIOD_RAD = math.pi / 2.0


def invalid_detection():
    return [0.0] * DETECTION_LENGTH


def invalid_tracked():
    return [0.0] * TRACKED_LENGTH


def invalid_landing_error():
    return [0.0] * LANDING_ERROR_LENGTH


def normalize_periodic_angle(angle, period=ANGLE_PERIOD_RAD):
    """Normalize an orientation to [-period/2, period/2)."""
    value = float(angle)
    if not math.isfinite(value) or period <= 0.0:
        raise ValueError('angle and period must be finite; period must be positive')
    return (value + period / 2.0) % period - period / 2.0


def periodic_angle_difference(target, reference, period=ANGLE_PERIOD_RAD):
    return normalize_periodic_angle(float(target) - float(reference), period)


def validate_detection(values):
    if len(values) != DETECTION_LENGTH:
        raise ValueError('detection must contain exactly 7 values')
    data = [float(value) for value in values]
    if not all(math.isfinite(value) for value in data):
        raise ValueError('all detection values must be finite')
    if data[VALID] not in (0.0, 1.0):
        raise ValueError('valid must be 0 or 1')
    if data[CENTER_X] < 0.0 or data[CENTER_Y] < 0.0:
        raise ValueError('target center cannot be negative')
    if not 0.0 <= data[CONFIDENCE] <= 100.0:
        raise ValueError('confidence must be in [0, 100]')
    if data[VALID] == 1.0:
        if data[OUTER_DIAMETER] <= 0.0 or data[INNER_DIAMETER] <= 0.0:
            raise ValueError('valid target diameters must be positive')
        if data[OUTER_DIAMETER] < data[INNER_DIAMETER]:
            raise ValueError('outer diameter must be at least inner diameter')
    elif data[OUTER_DIAMETER] < 0.0 or data[INNER_DIAMETER] < 0.0:
        raise ValueError('diameters cannot be negative')
    return data


def validate_tracked(values):
    if len(values) != TRACKED_LENGTH:
        raise ValueError('tracked target must contain exactly 12 values')
    data = [float(value) for value in values]
    if not all(math.isfinite(value) for value in data):
        raise ValueError('all tracked values must be finite')
    validate_detection(data[:DETECTION_LENGTH])
    if data[PREDICTED_CENTER_X] < 0.0 or data[PREDICTED_CENTER_Y] < 0.0:
        raise ValueError('predicted center cannot be negative')
    if data[TARGET_AGE_MS] < 0.0:
        raise ValueError('target age cannot be negative')
    return data


def validate_landing_error(values):
    if len(values) != LANDING_ERROR_LENGTH:
        raise ValueError('landing error must contain exactly 8 values')
    data = [float(value) for value in values]
    if not all(math.isfinite(value) for value in data):
        raise ValueError('all landing-error values must be finite')
    if data[VALID] not in (0.0, 1.0):
        raise ValueError('valid must be 0 or 1')
    if not 0.0 <= data[ERROR_CONFIDENCE] <= 100.0:
        raise ValueError('confidence must be in [0, 100]')
    if data[ERROR_TARGET_AGE_MS] < 0.0:
        raise ValueError('target age cannot be negative')
    return data
