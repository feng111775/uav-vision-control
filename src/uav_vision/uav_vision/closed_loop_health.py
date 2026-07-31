"""Dynamic, fail-closed readiness gate for the structured vision chain."""

from dataclasses import dataclass
import math


@dataclass
class HealthSample:
    now: float
    camera_open: bool
    frames_received: bool
    algorithm_alive: bool
    protocol_ok: bool
    ready_for_mission: bool
    performance_gate_passed: bool
    calibration_loaded: bool
    measured_fps: float
    last_measurement_age_ms: float
    receive_age_s: float
    recent_protocol_errors: int
    recent_duplicate_count: int
    recent_out_of_order_count: int
    recent_stalled: bool


class ClosedLoopHealthGate:
    """Require explicit enable, calibration and a stable measured window."""

    def __init__(self, min_fps=5.0, stable_seconds=2.0,
                 receive_timeout_seconds=0.30, source_age_limit_ms=300.0,
                 min_confidence=60.0):
        self.min_fps = float(min_fps)
        self.stable_seconds = float(stable_seconds)
        self.receive_timeout_seconds = float(receive_timeout_seconds)
        self.source_age_limit_ms = float(source_age_limit_ms)
        self.min_confidence = float(min_confidence)
        self.enabled = False
        self.stable_since = None
        self.stable_frame_count = 0
        self.ready_transitions = []
        self.last_ready = False

    def reset(self):
        self.stable_since = None
        self.stable_frame_count = 0
        self._set_ready(False, 0.0, 'reset')

    def observe_frame(self, now):
        if self.stable_since is None:
            self.stable_since = float(now)
            self.stable_frame_count = 1
        else:
            self.stable_frame_count += 1

    def evaluate(self, sample: HealthSample, closed_loop_enable=False):
        self.enabled = bool(closed_loop_enable)
        stable_time = (self.stable_since is not None and
                       sample.now - self.stable_since >= self.stable_seconds)
        required_frames = int(math.ceil(self.min_fps * self.stable_seconds))
        ready = all((self.enabled, sample.camera_open,
                     sample.frames_received, sample.algorithm_alive,
                     sample.protocol_ok, sample.ready_for_mission,
                     sample.performance_gate_passed,
                     sample.calibration_loaded,
                     sample.measured_fps >= self.min_fps,
                     sample.last_measurement_age_ms < self.source_age_limit_ms,
                     sample.receive_age_s <= self.receive_timeout_seconds,
                     stable_time, self.stable_frame_count >= required_frames,
                     sample.recent_protocol_errors == 0,
                     sample.recent_duplicate_count == 0,
                     sample.recent_out_of_order_count == 0,
                     not sample.recent_stalled))
        reason = 'ready' if ready else 'gate_closed'
        self._set_ready(ready, sample.now, reason)
        return ready

    def _set_ready(self, value, now, reason):
        value = bool(value)
        if value != self.last_ready:
            self.ready_transitions.append({
                'time': float(now), 'ready': value, 'reason': reason})
        self.last_ready = value
