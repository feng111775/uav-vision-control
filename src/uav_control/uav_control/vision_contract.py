"""Frozen D-task Float32MultiArray contracts used by the controller."""

import math


TRACKED_LENGTH = 12
LANDING_ERROR_LENGTH = 8

VALID = 0
ERROR_X_NORMALIZED = 1
ERROR_Y_NORMALIZED = 2
ERROR_CONFIDENCE = 6
ERROR_TARGET_AGE_MS = 7
TRACKED_CONFIDENCE = 6
TRACKED_TARGET_AGE_MS = 11


def invalid_tracked():
    return [0.0] * TRACKED_LENGTH


def invalid_landing_error():
    return [0.0] * LANDING_ERROR_LENGTH


def _finite_array(values, length, label):
    if len(values) < length:
        raise ValueError(f'{label} must contain at least {length} values')
    data = [float(value) for value in values]
    if not all(math.isfinite(value) for value in data):
        raise ValueError(f'{label} values must be finite')
    if data[VALID] < 0.0:
        raise ValueError('valid cannot be negative')
    return data


def validate_tracked(values):
    data = _finite_array(values, TRACKED_LENGTH, 'tracked target')
    if not 0.0 <= data[6] <= 100.0:
        raise ValueError('confidence must be in [0, 100]')
    if data[11] < 0.0:
        raise ValueError('target age cannot be negative')
    return data


def validate_landing_error(values):
    data = _finite_array(values, LANDING_ERROR_LENGTH, 'landing error')
    if not 0.0 <= data[ERROR_CONFIDENCE] <= 100.0:
        raise ValueError('confidence must be in [0, 100]')
    if data[ERROR_TARGET_AGE_MS] < 0.0:
        raise ValueError('target age cannot be negative')
    return data
