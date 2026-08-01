# flake8: noqa
"""Formal first-task mission state machine for the real aircraft path."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .mission_schema import CarProgress


FIRST_TASK_STATES = (
    "WAIT_START",
    "TAKEOFF",
    "HOVER_150CM",
    "HOVER_3S",
    "SEARCH_CAR",
    "VISION_FOLLOW",
    "ALIGN_FOR_DROP",
    "PAYLOAD_RELEASE",
    "WAIT_RELEASE_ACK",
    "RETURN_HOME",
    "FINAL_LAND",
    "COMPLETE",
    "FAILSAFE_LAND",
    "TIMEOUT",
)


@dataclass
class FirstTaskInputs:
    """Read-only inputs from PX4, vision, car link, and readiness."""

    now: float
    start_signal: bool = False
    readiness: bool = False
    px4_fresh: bool = False
    position_valid: bool = False
    heading_valid: bool = False
    armed: bool = False
    offboard: bool = False
    failsafe: bool = False
    x: float | None = None
    y: float | None = None
    z: float | None = None
    vx: float | None = None
    vy: float | None = None
    vz: float | None = None
    heading: float | None = None
    vision_ready: bool = False
    tracked_valid: bool = False
    landing_valid: bool = False
    capture_stamp_valid: bool = False
    frame_sequence_new: bool = False
    vision_fresh: bool = False
    confidence_ok: bool = False
    error_x_norm: float | None = None
    error_y_norm: float | None = None
    target_age_ms: float = 1e9
    car_progress: int = 0
    release_ack: bool = False
    landed: bool = False
    disarmed: bool = False


class FirstTaskMissionLogic:
    """Nominal first-task sequence with fail-closed timeout handling."""

    def __init__(
        self,
        target_altitude_m: float = 1.5,
        hover_duration_s: float = 3.0,
        search_speed_m_s: float = 0.18,
        mission_total_timeout_s: float = 90.0,
        takeoff_timeout_s: float = 12.0,
        search_deadline_s: float = 20.0,
        payload_deadline_s: float = 35.0,
        return_home_deadline_s: float = 60.0,
        forced_land_deadline_s: float = 80.0,
        land_confirm_timeout_s: float = 4.0,
        disarm_confirm_timeout_s: float = 4.0,
        closed_loop_min_confidence: float = 60.0,
        closed_loop_min_age_ms: float = 300.0,
        b_progress_value: int = int(CarProgress.PASSED_B),
        d_progress_value: int = int(CarProgress.PASSED_D),
        enable_control: bool = False,
        enable_auto_arm: bool = False,
        enable_payload_release: bool = False,
        enable_closed_loop: bool = False,
        profile_name: str = "readonly_bench",
    ) -> None:
        self.target_altitude_m = float(target_altitude_m)
        self.hover_duration_s = float(hover_duration_s)
        self.search_speed_m_s = float(search_speed_m_s)
        self.mission_total_timeout_s = float(mission_total_timeout_s)
        self.takeoff_timeout_s = float(takeoff_timeout_s)
        self.search_deadline_s = float(search_deadline_s)
        self.payload_deadline_s = float(payload_deadline_s)
        self.return_home_deadline_s = float(return_home_deadline_s)
        self.forced_land_deadline_s = float(forced_land_deadline_s)
        self.land_confirm_timeout_s = float(land_confirm_timeout_s)
        self.disarm_confirm_timeout_s = float(disarm_confirm_timeout_s)
        self.closed_loop_min_confidence = float(closed_loop_min_confidence)
        self.closed_loop_min_age_ms = float(closed_loop_min_age_ms)
        self.b_progress_value = int(b_progress_value)
        self.d_progress_value = int(d_progress_value)
        self.enable_control = bool(enable_control)
        self.enable_auto_arm = bool(enable_auto_arm)
        self.enable_payload_release = bool(enable_payload_release)
        self.enable_closed_loop = bool(enable_closed_loop)
        self.profile_name = str(profile_name)
        self.reset()

    def reset(self) -> None:
        self.state = "WAIT_START"
        self.state_since = 0.0
        self.started_at: float | None = None
        self.home: tuple[float, float, float, float] | None = None
        self.home_captured_at: float | None = None
        self.release_attempted = False
        self.release_ack = False
        self.search_speed = 0.0
        self.last_reason = "RESET"
        self.last_target_sequence: int | None = None
        self.last_landing_sequence: int | None = None
        self._hover_entered_at: float | None = None
        self._landed_since: float | None = None
        self._disarmed_since: float | None = None

    def transition(self, state: str, now: float, reason: str | None = None) -> None:
        self.state = state
        self.state_since = float(now)
        if reason:
            self.last_reason = reason
        if state == "HOVER_3S":
            self._hover_entered_at = float(now)
        if state == "FINAL_LAND":
            self._landed_since = None
            self._disarmed_since = None

    def _timeout(self, now: float) -> bool:
        if self.started_at is None:
            return False
        if now - self.started_at > self.mission_total_timeout_s:
            self.transition("TIMEOUT", now, "MISSION_TOTAL_TIMEOUT")
            return True
        return False

    @staticmethod
    def _finite(*values: float | None) -> bool:
        return all(v is not None and math.isfinite(float(v)) for v in values)

    def _position(self, inputs: FirstTaskInputs) -> tuple[float, float, float] | None:
        if not inputs.position_valid or not self._finite(inputs.x, inputs.y, inputs.z):
            return None
        return float(inputs.x), float(inputs.y), float(inputs.z)

    def _height_error(self, inputs: FirstTaskInputs) -> float | None:
        if self.home is None or not self._position(inputs):
            return None
        return float(self.home[2] + self.target_altitude_m - float(inputs.z))

    def altitude_reached(self, inputs: FirstTaskInputs) -> bool:
        if self.home is None or not inputs.position_valid:
            return False
        pos = self._position(inputs)
        if pos is None or not self._finite(inputs.vx, inputs.vy, inputs.vz):
            return False
        target_z = self.home[2] + self.target_altitude_m
        return (
            abs(pos[2] - target_z) <= 0.08
            and math.hypot(float(inputs.vx), float(inputs.vy)) <= 0.20
            and abs(float(inputs.vz)) <= 0.18
        )

    def home_reached(self, inputs: FirstTaskInputs) -> bool:
        if self.home is None:
            return False
        pos = self._position(inputs)
        if pos is None or not self._finite(inputs.vx, inputs.vy, inputs.vz):
            return False
        dx = pos[0] - self.home[0]
        dy = pos[1] - self.home[1]
        dz = pos[2] - self.home[2]
        return (
            math.hypot(dx, dy) <= 0.25
            and abs(dz) <= 0.12
            and math.hypot(float(inputs.vx), float(inputs.vy)) <= 0.20
            and abs(float(inputs.vz)) <= 0.18
        )

    def closed_loop_permitted(self, inputs: FirstTaskInputs) -> bool:
        return bool(
            self.enable_control
            and self.enable_closed_loop
            and inputs.readiness
            and inputs.vision_ready
            and inputs.tracked_valid
            and inputs.landing_valid
            and inputs.capture_stamp_valid
            and inputs.frame_sequence_new
            and inputs.vision_fresh
            and inputs.confidence_ok
            and not inputs.failsafe
            and inputs.px4_fresh
            and inputs.position_valid
            and inputs.heading_valid
        )

    def _start_mission(self, inputs: FirstTaskInputs) -> None:
        self.started_at = float(inputs.now)
        self.transition("TAKEOFF", inputs.now, "MISSION_STARTED")

    def _ready_to_hover(self, inputs: FirstTaskInputs) -> bool:
        return self.altitude_reached(inputs)

    def step(self, inputs: FirstTaskInputs) -> None:
        now = float(inputs.now)
        if self.state == "COMPLETE":
            return
        if inputs.failsafe:
            self.transition("FAILSAFE_LAND", now, "PX4_FAILSAFE")
            return
        if self._timeout(now):
            return
        if self.home is None and self._position(inputs) is not None:
            x, y, z = self._position(inputs)
            self.home = (x, y, z, float(inputs.heading or 0.0))
            self.home_captured_at = now
        if self.state == "WAIT_START":
            if inputs.start_signal and inputs.readiness and self.home is not None:
                self._start_mission(inputs)
            return
        if self.started_at is None:
            return
        if self.state == "TAKEOFF":
            if now - self.started_at > self.takeoff_timeout_s:
                self.transition("TIMEOUT", now, "TAKEOFF_TIMEOUT")
            elif self._ready_to_hover(inputs):
                self.transition("HOVER_150CM", now, "TAKEOFF_COMPLETE")
            return
        if self.state == "HOVER_150CM":
            if self._ready_to_hover(inputs):
                self.transition("HOVER_3S", now, "HOVER_STABLE")
            return
        if self.state == "HOVER_3S":
            if now - self.state_since >= self.hover_duration_s:
                self.transition("SEARCH_CAR", now, "HOVER_DONE")
            return
        if self.state == "SEARCH_CAR":
            self.search_speed = min(self.search_speed_m_s, 0.18)
            if now - self.started_at > self.search_deadline_s:
                self.transition("RETURN_HOME", now, "SEARCH_DEADLINE")
            elif inputs.car_progress >= self.b_progress_value and self.closed_loop_permitted(inputs):
                self.transition("VISION_FOLLOW", now, "CAR_B_REACHED")
            return
        if self.state == "VISION_FOLLOW":
            if now - self.started_at > self.payload_deadline_s:
                self.transition("RETURN_HOME", now, "PAYLOAD_DEADLINE")
            elif inputs.car_progress >= self.d_progress_value:
                self.transition("ALIGN_FOR_DROP", now, "CAR_D_REACHED")
            elif not self.closed_loop_permitted(inputs):
                self.transition("SEARCH_CAR", now, "VISION_LOST")
            return
        if self.state == "ALIGN_FOR_DROP":
            if now - self.started_at > self.payload_deadline_s:
                self.transition("RETURN_HOME", now, "PAYLOAD_DEADLINE")
            elif self.enable_payload_release and not self.release_attempted:
                self.transition("PAYLOAD_RELEASE", now, "PAYLOAD_RELEASE")
                self.release_attempted = True
            elif not self.closed_loop_permitted(inputs):
                self.transition("RETURN_HOME", now, "ALIGN_ABORT")
            return
        if self.state == "PAYLOAD_RELEASE":
            if inputs.release_ack:
                self.release_ack = True
                self.transition("WAIT_RELEASE_ACK", now, "PAYLOAD_ACK")
            elif now - self.state_since > max(0.5, self.land_confirm_timeout_s):
                self.transition("WAIT_RELEASE_ACK", now, "PAYLOAD_WAIT_ACK")
            return
        if self.state == "WAIT_RELEASE_ACK":
            if self.release_ack:
                self.transition("RETURN_HOME", now, "PAYLOAD_CONFIRMED")
            elif now - self.state_since > self.land_confirm_timeout_s:
                self.transition("RETURN_HOME", now, "PAYLOAD_ACK_TIMEOUT")
            return
        if self.state == "RETURN_HOME":
            if now - self.started_at > self.return_home_deadline_s:
                self.transition("FINAL_LAND", now, "RETURN_HOME_DEADLINE")
            elif self.home_reached(inputs):
                self.transition("FINAL_LAND", now, "HOME_REACHED")
            return
        if self.state == "FINAL_LAND":
            if inputs.landed:
                if self._landed_since is None:
                    self._landed_since = now
                if inputs.disarmed and self._disarmed_since is None:
                    self._disarmed_since = now
                if (
                    self._landed_since is not None
                    and now - self._landed_since >= self.land_confirm_timeout_s
                    and self._disarmed_since is not None
                    and now - self._disarmed_since >= self.disarm_confirm_timeout_s
                ):
                    self.transition("COMPLETE", now, "MISSION_COMPLETE")
            else:
                self._landed_since = None
            if now - self.started_at > self.forced_land_deadline_s:
                self.last_reason = "FORCED_LAND_DEADLINE"
            return
        if self.state == "FAILSAFE_LAND":
            if inputs.landed and inputs.disarmed:
                self.transition("TIMEOUT", now, "FAILSAFE_COMPLETE")
