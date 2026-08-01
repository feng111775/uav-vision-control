"""ROS-independent continuous PX4 readiness evaluation."""

import math


class ReadinessGate:
    """Require all PX4 health conditions for one uninterrupted window."""

    STREAMS = ('status', 'position', 'attitude')

    def __init__(self, stable_seconds=3.0, status_timeout=1.0,
                 position_timeout=0.5, attitude_timeout=0.5,
                 simulation_mode=False,
                 allow_sitl_heading_quality_bypass=False):
        self.stable_seconds = float(stable_seconds)
        self.timeouts = {
            'status': float(status_timeout),
            'position': float(position_timeout),
            'attitude': float(attitude_timeout),
        }
        self.simulation_mode = bool(simulation_mode)
        self.heading_bypass = bool(allow_sitl_heading_quality_bypass)
        if self.heading_bypass and not self.simulation_mode:
            raise ValueError('heading quality bypass is SITL-only')
        self.received = {name: None for name in self.STREAMS}
        self.valid = {name: False for name in self.STREAMS}
        self.stable_since = None
        self.last_reasons = ()
        self.failsafe = False
        self.disarmed = False

    def update_status(self, now, preflight_ok, failsafe, disarmed):
        self.received['status'] = float(now)
        self.failsafe = bool(failsafe)
        self.disarmed = bool(disarmed)
        self.valid['status'] = bool(
            preflight_ok and not failsafe and disarmed)

    def update_position(self, now, xy_valid, z_valid, v_xy_valid,
                        v_z_valid, heading_good, heading):
        values_valid = all(
            (xy_valid, z_valid, v_xy_valid, v_z_valid))
        heading_valid = (
            bool(heading_good) or
            (self.simulation_mode and self.heading_bypass))
        self.received['position'] = float(now)
        self.valid['position'] = bool(
            values_valid and heading_valid and math.isfinite(float(heading)))

    def update_attitude(self, now, quaternion, norm_tolerance=0.1):
        try:
            values = tuple(float(value) for value in quaternion)
        except (TypeError, ValueError):
            values = ()
        finite = len(values) == 4 and all(math.isfinite(v) for v in values)
        norm = math.sqrt(sum(v * v for v in values)) if finite else 0.0
        self.received['attitude'] = float(now)
        self.valid['attitude'] = bool(
            finite and abs(norm - 1.0) <= float(norm_tolerance))

    def evaluate(self, now, topics_present=None):
        now = float(now)
        present = (
            {name: True for name in self.STREAMS}
            if topics_present is None else topics_present)
        reasons = []
        for name in self.STREAMS:
            if not present.get(name, False):
                reasons.append(name + '_topic_missing')
            received = self.received[name]
            if received is None:
                reasons.append(name + '_never_received')
            elif now - received > self.timeouts[name]:
                reasons.append(name + '_stale')
            if not self.valid[name]:
                reasons.append(name + '_invalid')
        healthy = not reasons
        if not healthy:
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now
        ready = (
            healthy and self.stable_since is not None and
            now - self.stable_since >= self.stable_seconds)
        self.last_reasons = tuple(reasons)
        return ready, self.last_reasons
