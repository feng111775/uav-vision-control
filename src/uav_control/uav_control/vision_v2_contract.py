"""Adapter for accepted structured vision V2 messages."""

# Only this module knows the V2 message fields.  The mission state machine
# consumes the normalized observation and never reads invalid metric fields.

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VisionV2Observation:
    received_monotonic: float
    source_timestamp: float | None
    frame_sequence: int | None
    target_valid: bool
    confidence: float
    error_x_norm: float
    error_y_norm: float
    metric_valid: bool
    source_alive: bool
    health_valid: bool
    fresh: bool
    rejection_reason: str = ''

    @property
    def control_allowed(self):
        return (self.source_alive and self.health_valid and self.fresh and
                self.target_valid and math.isfinite(self.confidence) and
                math.isfinite(self.error_x_norm) and
                math.isfinite(self.error_y_norm))


class SequenceGate:
    """Accept new uint32 sequence values, including wrap and restart."""

    def __init__(self):
        self.last = None
        self.restart_generation = 0
        self.reset_pending = False
        self.duplicate_count = 0
        self.out_of_order_count = 0

    def reset(self):
        self.__init__()

    def accept(self, value):
        value = int(value) & 0xffffffff
        if self.last is None:
            self.last = value
            return True, 'initial'
        delta = (value - self.last) & 0xffffffff
        if delta == 0:
            self.duplicate_count += 1
            return False, 'duplicate'
        if delta < 0x80000000:
            self.last = value
            return True, 'new'
        # A sender restart conventionally begins at sequence one.  Accept it
        # as a new generation but require fresh confirmations before control.
        if value == 1 and self.last > 1:
            self.last = value
            self.restart_generation += 1
            self.reset_pending = True
            return True, 'restart'
        if self.last >= 0xf0000000 and value <= 0x0fffffff:
            self.last = value
            return True, 'wrap'
        self.out_of_order_count += 1
        return False, 'out_of_order'


def _stamp_seconds(stamp):
    try:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    except (AttributeError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) and value > 0.0 else None


class VisionV2Adapter:
    """Validate and combine TargetObservation, LandingError and health."""

    def __init__(self, min_confidence=60.0, require_ready_for_closed_loop=True):
        self.min_confidence = float(min_confidence)
        self.require_ready_for_closed_loop = bool(require_ready_for_closed_loop)
        self.sequence = SequenceGate()
        self.post_restart_frames = 0
        self.landing = None
        self.tracked = None
        self.health = None
        self.last_receive = None
        self.last_reason = 'not_received'

    def update_health(self, message, received_monotonic):
        self.health = (message, float(received_monotonic))

    def update_landing(self, message, received_monotonic):
        return self._update(message, received_monotonic, 'landing')

    def update_tracked(self, message, received_monotonic):
        return self._update(message, received_monotonic, 'tracked')

    def _update(self, message, received_monotonic, kind):
        try:
            sequence = int(message.frame_sequence) & 0xffffffff
        except (AttributeError, TypeError, ValueError):
            self.last_reason = 'missing_sequence'
            return False
        already_same = ((kind == 'landing' and self.landing is not None and
                         int(self.landing[0].frame_sequence) == sequence) or
                        (kind == 'tracked' and self.tracked is not None and
                         int(self.tracked[0].frame_sequence) == sequence))
        paired = (not already_same and
                  ((kind == 'landing' and self.tracked is not None and
                    int(self.tracked[0].frame_sequence) == sequence) or
                   (kind == 'tracked' and self.landing is not None and
                    int(self.landing[0].frame_sequence) == sequence)))
        if already_same:
            self.sequence.duplicate_count += 1
            self.last_reason = 'duplicate'
            return False
        accepted, reason = (True, 'paired') if paired else self.sequence.accept(sequence)
        if not accepted:
            self.last_reason = reason
            return False
        if reason == 'restart':
            self.landing = None
            self.tracked = None
            self.post_restart_frames = 1
        elif reason == 'new' and self.sequence.reset_pending:
            self.post_restart_frames += 1
            if self.post_restart_frames >= 3:
                self.sequence.reset_pending = False
        if kind == 'landing':
            self.landing = (message, float(received_monotonic))
        else:
            self.tracked = (message, float(received_monotonic))
        self.last_receive = float(received_monotonic)
        self.last_reason = ''
        return True

    def observation(self, now, receive_timeout=0.3, source_age_limit=0.3,
                    source_now=None):
        now = float(now)
        if self.landing is None or self.tracked is None:
            return self._invalid(now, 'missing_observation')
        landing, landing_received = self.landing
        tracked, tracked_received = self.tracked
        if int(landing.frame_sequence) != int(tracked.frame_sequence):
            return self._invalid(now, 'sequence_mismatch')
        latest = max(landing_received, tracked_received)
        source_alive = 0.0 <= now - latest <= float(receive_timeout)
        health_valid = self._health_valid(now, receive_timeout)
        if self.sequence.reset_pending:
            health_valid = False
        source_stamp = _stamp_seconds(landing.header.stamp)
        source_clock = now if source_now is None else float(source_now)
        source_fresh = (source_stamp is not None and
                        0.0 <= source_clock - source_stamp <=
                        float(source_age_limit))
        finite = all(math.isfinite(float(value)) for value in (
            landing.error_x_norm, landing.error_y_norm,
            landing.confidence, tracked.error_x_norm, tracked.error_y_norm,
            tracked.confidence))
        valid = bool(landing.valid and landing.capture_stamp_valid and
                     tracked.measurement_valid and tracked.confirmed)
        confidence = min(float(landing.confidence), float(tracked.confidence))
        accepted = (source_alive and health_valid and source_fresh and finite and
                    valid and confidence >= self.min_confidence)
        reason = '' if accepted else self._reason(
            source_alive, health_valid, source_fresh, finite, valid, confidence)
        return VisionV2Observation(
            latest, source_stamp, int(landing.frame_sequence), accepted,
            confidence, float(landing.error_x_norm),
            float(landing.error_y_norm), bool(landing.metric_valid),
            source_alive, health_valid, source_fresh, reason)

    def _health_valid(self, now, timeout):
        if self.health is None:
            return False
        message, received = self.health
        if now - received < 0.0 or now - received > float(timeout):
            return False
        fields = ('camera_open', 'frames_received', 'algorithm_alive',
                  'protocol_ok')
        healthy = all(bool(getattr(message, field, False)) for field in fields)
        if self.require_ready_for_closed_loop:
            healthy = healthy and bool(message.ready_for_closed_loop)
        return healthy

    @staticmethod
    def _reason(source_alive, health_valid, source_fresh, finite, valid,
                confidence):
        if not source_alive:
            return 'receive_timeout'
        if not health_valid:
            return 'health_invalid_or_stale'
        if not source_fresh:
            return 'source_timestamp_stale'
        if not finite:
            return 'non_finite'
        if not valid:
            return 'measurement_invalid'
        if confidence < 0.0:
            return 'confidence_invalid'
        return 'confidence_below_threshold'

    def _invalid(self, now, reason):
        return VisionV2Observation(
            now, None, None, False, 0.0, 0.0, 0.0, False, False, False,
            False, reason)
