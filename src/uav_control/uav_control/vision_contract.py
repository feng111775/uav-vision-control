"""
Legacy vision adapter and ROS-independent internal observation model.

The array indexes below are intentionally isolated to this module.  When the
formal V2 message arrives, replace ``LegacyVisionAdapter`` with a new adapter
without changing the mission state machine.
"""

from dataclasses import dataclass
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


@dataclass(frozen=True)
class VisionObservation:
    """Normalized vision state consumed by control code."""

    received_monotonic_time: float | None
    source_timestamp: float | None
    target_valid: bool
    confidence: float
    horizontal_error_x: float
    horizontal_error_y: float
    target_age: float
    finite_values: bool
    source_alive: bool
    health_valid: bool
    rejection_reason: str

    @property
    def control_allowed(self):
        """Return true only when this observation is safe for control."""
        return (self.source_alive and self.health_valid and self.target_valid
                and self.finite_values)


class LegacyVisionAdapter:
    """Adapt the temporary Float32MultiArray interface into one model."""

    def __init__(self):
        self._landing = None
        self._tracked = None
        self._landing_received = None
        self._tracked_received = None
        self._landing_reason = 'landing_not_received'
        self._tracked_reason = 'tracked_not_received'

    def update_landing(self, values, received_monotonic_time):
        self._landing_received = float(received_monotonic_time)
        try:
            self._landing = validate_landing_error(values)
            self._landing_reason = ''
        except (TypeError, ValueError) as error:
            self._landing = None
            self._landing_reason = str(error)

    def update_tracked(self, values, received_monotonic_time):
        self._tracked_received = float(received_monotonic_time)
        try:
            self._tracked = validate_tracked(values)
            self._tracked_reason = ''
        except (TypeError, ValueError) as error:
            self._tracked = None
            self._tracked_reason = str(error)

    def observation(self, now, receive_timeout_seconds=0.3,
                    source_age_limit_seconds=0.3, min_confidence=60.0,
                    health_valid=True):
        """Return a fresh normalized observation without retaining old errors."""
        now = float(now)
        timeout = float(receive_timeout_seconds)
        source_limit = float(source_age_limit_seconds)
        landing = self._landing
        tracked = self._tracked
        latest_received = max(
            [value for value in (self._landing_received, self._tracked_received)
             if value is not None], default=None)
        source_alive = (latest_received is not None and
                        0.0 <= now - latest_received <= timeout)
        if landing is None or tracked is None:
            reason = self._landing_reason or self._tracked_reason
            return VisionObservation(
                latest_received, None, False, 0.0, 0.0, 0.0, float('inf'),
                False, source_alive, bool(health_valid), reason or 'invalid_frame')
        received_age = max(
            now - float(self._landing_received),
            now - float(self._tracked_received))
        age_ms = max(landing[ERROR_TARGET_AGE_MS],
                     tracked[TRACKED_TARGET_AGE_MS])
        finite = all(math.isfinite(value) for value in (
            landing[ERROR_X_NORMALIZED], landing[ERROR_Y_NORMALIZED],
            landing[ERROR_CONFIDENCE], tracked[TRACKED_CONFIDENCE], age_ms))
        target_valid = (landing[VALID] >= 0.5 and tracked[VALID] >= 0.5 and
                        min(landing[ERROR_CONFIDENCE],
                            tracked[TRACKED_CONFIDENCE]) >= min_confidence and
                        age_ms >= 0.0 and age_ms <= source_limit * 1000.0 and
                        received_age >= 0.0 and received_age <= timeout)
        reason = ''
        if not source_alive:
            reason = 'source_timeout'
        elif not health_valid:
            reason = 'health_invalid'
        elif not finite:
            reason = 'non_finite'
        elif not target_valid:
            reason = 'target_invalid_or_stale'
        return VisionObservation(
            latest_received, None, target_valid, min(
                landing[ERROR_CONFIDENCE], tracked[TRACKED_CONFIDENCE]),
            landing[ERROR_X_NORMALIZED], landing[ERROR_Y_NORMALIZED],
            age_ms / 1000.0, finite, source_alive, bool(health_valid), reason)


def invalid_tracked():
    return [0.0] * TRACKED_LENGTH


def invalid_landing_error():
    return [0.0] * LANDING_ERROR_LENGTH


def make_legacy_no_target():
    """Return matching legacy arrays for an online camera with no target."""
    tracked = [0.0] * TRACKED_LENGTH
    landing = [0.0] * LANDING_ERROR_LENGTH
    return tracked, landing


def make_legacy_aligned(error_x=0.0, error_y=0.0, confidence=95.0,
                        target_age_ms=20.0):
    """Build deterministic, valid legacy arrays for simulation only."""
    tracked = [1.0, 0.0, 0.0, 20.0, 10.0, 0.0, confidence, 0.0, 0.0,
               0.0, 0.0, target_age_ms]
    landing = [1.0, error_x, error_y, error_x * 640.0, error_y * 480.0,
               0.0, confidence, target_age_ms]
    return tracked, landing


def make_legacy_malformed():
    """Return intentionally invalid arrays for adapter rejection tests."""
    return [1.0, float('nan')], [1.0, 0.0, float('inf')]


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
