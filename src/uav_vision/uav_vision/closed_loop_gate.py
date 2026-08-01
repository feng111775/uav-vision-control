"""Fail-closed, hysteretic gate for formal V2 visual closed loop."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GateSample:
    enabled: bool
    camera_open: bool
    frames_received: bool
    algorithm_alive: bool
    protocol_ok: bool
    performance_ok: bool
    orientation_ok: bool
    mapping_ok: bool
    camera_mount_profile: str
    disconnected: bool = False
    stale: bool = False
    data_age_ok: bool = False
    capture_stamp_ok: bool = False
    sequence_gap_severe: bool = False


class ClosedLoopGate:
    """Open only after stable warmup; close immediately on hard faults."""

    def __init__(
        self,
        warmup_sec: float = 3.0,
        warmup_frames: int = 30,
        max_recent_drop_count: int = 3,
        max_recent_protocol_errors: int = 0,
        soft_failure_count: int = 3,
    ):
        self.warmup_sec = float(warmup_sec)
        self.warmup_frames = int(warmup_frames)
        self.max_recent_drop_count = int(max_recent_drop_count)
        self.max_recent_protocol_errors = int(max_recent_protocol_errors)
        self.soft_failure_count = int(soft_failure_count)
        self.reset()

    def reset(self):
        self.ready = False
        self.first_good_time = None
        self.good_frames = 0
        self.soft_failures = 0
        self.switch_count = 0
        self.would_be_ready_count = 0
        self.total_count = 0

    def update(self, now: float, sample: GateSample) -> bool:
        self.total_count += 1
        hard_fault = (
            not sample.enabled
            or not sample.camera_open
            or not sample.frames_received
            or not sample.algorithm_alive
            or not sample.protocol_ok
            or not sample.orientation_ok
            or not sample.mapping_ok
            or sample.camera_mount_profile == "unverified"
            or sample.disconnected
            or sample.stale
            or not sample.data_age_ok
            or not sample.capture_stamp_ok
            or sample.sequence_gap_severe
        )
        soft_ok = (
            sample.performance_ok
            and sample.camera_mount_profile != "unverified"
        )
        would_be_ready = not hard_fault and soft_ok
        self.would_be_ready_count += int(would_be_ready)
        if hard_fault:
            self._close()
            return False
        if not soft_ok:
            self.soft_failures += 1
            if self.soft_failures >= self.soft_failure_count:
                self._close()
            return self.ready
        self.soft_failures = 0
        if self.first_good_time is None:
            self.first_good_time = float(now)
            self.good_frames = 0
        self.good_frames += 1
        if (
            not self.ready
            and float(now) - self.first_good_time >= self.warmup_sec
            and self.good_frames >= self.warmup_frames
        ):
            self.ready = True
            self.switch_count += 1
        return self.ready

    def _close(self):
        if self.ready:
            self.switch_count += 1
        self.ready = False
        self.first_good_time = None
        self.good_frames = 0
        self.soft_failures = 0

