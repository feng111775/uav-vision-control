"""Pure, deterministic stage 4C mission logic."""

from dataclasses import dataclass
from enum import Enum
import math


class MissionState(str, Enum):
    """Competition mission states."""

    INITIALIZING = 'INITIALIZING'
    WAIT_FOR_START = 'WAIT_FOR_START'
    TAKEOFF = 'TAKEOFF'
    HOVER_STABLE = 'HOVER_STABLE'
    TRANSIT_TO_INTERCEPT = 'TRANSIT_TO_INTERCEPT'
    SEARCH_CAR = 'SEARCH_CAR'
    ACQUIRE_CAR = 'ACQUIRE_CAR'
    FOLLOW_CAR = 'FOLLOW_CAR'
    DROP_ALIGN = 'DROP_ALIGN'
    RELEASE_PAYLOAD = 'RELEASE_PAYLOAD'
    RETURN_HOME = 'RETURN_HOME'
    LAND = 'LAND'
    COMPLETE = 'COMPLETE'
    ABORT_RETURN = 'ABORT_RETURN'
    EMERGENCY_LAND = 'EMERGENCY_LAND'


@dataclass
class MissionConfig:
    """Safety and timing limits for the pure mission flow."""

    takeoff_height_m: float = 1.5
    takeoff_height_tolerance_m: float = 0.1
    stable_hover_duration_s: float = 3.0
    car_speed_mps: float = 0.1
    mission_timeout_s: float = 90.0
    target_max_age_s: float = 0.5
    target_confirm_frames: int = 3
    target_short_loss_s: float = 0.5
    target_long_loss_s: float = 2.0
    follow_stable_duration_s: float = 2.0
    drop_alignment_tolerance: float = 0.08
    return_home_tolerance_m: float = 0.2
    home_stable_duration_s: float = 1.0
    takeoff_timeout_s: float = 15.0
    transit_timeout_s: float = 15.0
    search_timeout_s: float = 15.0
    acquire_timeout_s: float = 5.0
    follow_timeout_s: float = 15.0
    release_timeout_s: float = 3.0
    return_timeout_s: float = 20.0
    land_timeout_s: float = 20.0
    max_horizontal_speed: float = 0.5
    max_vertical_speed: float = 0.3
    intercept_x_m: float = 1.0
    intercept_y_m: float = 0.0
    field_yaw_rad: float = 0.0
    field_offset_x_m: float = 0.0
    field_offset_y_m: float = 0.0
    intercept_tolerance_m: float = 0.25
    px4_data_timeout_s: float = 1.0


@dataclass
class MarkerObservation:
    """Timestamped circular-cross marker observation."""

    stamp: float
    detected: bool
    confidence: float
    error_x: float
    error_y: float
    relative_x: float | None = None
    relative_y: float | None = None
    velocity_x: float | None = None
    velocity_y: float | None = None

    def fresh(self, now: float, max_age: float) -> bool:
        """Return whether this observation is finite, detected and fresh."""
        values = (self.stamp, self.confidence, self.error_x, self.error_y)
        return (self.detected and all(math.isfinite(v) for v in values) and
                0.0 <= now - self.stamp <= max_age)


class StartGate:
    """Accept each task identifier once, without triggering flight itself."""

    def __init__(self):
        self.accepted = set()
        self.closed = False

    def accept(self, task_id: str, valid: bool) -> bool:
        """Accept a new valid identifier unless the gate is closed."""
        if self.closed or not valid or not task_id or task_id in self.accepted:
            return False
        self.accepted.add(task_id)
        return True


class PayloadGate:
    """One-shot dry-run payload interlock."""

    def __init__(self, dry_run: bool = True):
        self.dry_run = dry_run
        self.release_ids = set()
        self.released_tasks = set()
        self.hardware_access_count = 0

    def request(self, task_id: str, release_id: str, allowed: bool) -> str:
        """Return SUCCESS or REJECTED without touching hardware."""
        if (not allowed or not task_id or not release_id or
                release_id in self.release_ids or
                task_id in self.released_tasks):
            return 'REJECTED'
        self.release_ids.add(release_id)
        if not self.dry_run:
            return 'FAILED'
        self.released_tasks.add(task_id)
        return 'SUCCESS'


class MissionFlow:
    """Pure high-level mission state machine; it never publishes to PX4."""

    TIMEOUTS = {
        MissionState.TAKEOFF: 'takeoff_timeout_s',
        MissionState.TRANSIT_TO_INTERCEPT: 'transit_timeout_s',
        MissionState.SEARCH_CAR: 'search_timeout_s',
        MissionState.ACQUIRE_CAR: 'acquire_timeout_s',
        MissionState.FOLLOW_CAR: 'follow_timeout_s',
        MissionState.RELEASE_PAYLOAD: 'release_timeout_s',
        MissionState.RETURN_HOME: 'return_timeout_s',
        MissionState.ABORT_RETURN: 'return_timeout_s',
        MissionState.LAND: 'land_timeout_s',
    }

    def __init__(self, config: MissionConfig | None = None):
        self.config = config or MissionConfig()
        self.state = MissionState.INITIALIZING
        self.state_since = 0.0
        self.task_started = None
        self.task_id = ''
        self.home = None
        self.hover_since = None
        self.home_since = None
        self.follow_since = None
        self.target_lost_since = None
        self.confirm_frames = 0
        self.payload_requested = False
        self.last_valid_observation = None
        self.last_reason = 'constructed'

    def transition(self, state: MissionState, now: float, reason: str):
        """Move state and reset state-specific continuous timers."""
        self.state = state
        self.state_since = now
        self.last_reason = reason
        if state != MissionState.HOVER_STABLE:
            self.hover_since = None
        if state not in (MissionState.FOLLOW_CAR, MissionState.DROP_ALIGN):
            self.follow_since = None

    def initialize(self, now: float):
        """Finish initialization without starting a task."""
        self.transition(MissionState.WAIT_FOR_START, now, 'initialized')

    def start(self, task_id: str, now: float, home: tuple[float, float, float]):
        """Start once from WAIT_FOR_START and record immutable home."""
        if self.state != MissionState.WAIT_FOR_START or not task_id:
            return False
        self.task_id, self.task_started, self.home = task_id, now, tuple(home)
        self.transition(MissionState.TAKEOFF, now, 'valid_start')
        return True

    def predicted_distance(self, now: float) -> float:
        """Compute car distance using s = speed times elapsed time."""
        elapsed = 0.0 if self.task_started is None else now - self.task_started
        return self.config.car_speed_mps * max(0.0, elapsed)

    def field_to_local(self, x: float, y: float) -> tuple[float, float]:
        """Rotate and offset a field-frame point into PX4 local coordinates."""
        cosine = math.cos(self.config.field_yaw_rad)
        sine = math.sin(self.config.field_yaw_rad)
        return (
            self.config.field_offset_x_m + cosine * x - sine * y,
            self.config.field_offset_y_m + sine * x + cosine * y,
        )

    def target_valid(self, observation: MarkerObservation | None,
                     now: float) -> bool:
        """Reject stale observations."""
        return bool(observation and
                    observation.fresh(now, self.config.target_max_age_s))

    def _elapsed(self, now: float) -> float:
        return 0.0 if self.task_started is None else now - self.task_started

    def _timed_out(self, now: float) -> bool:
        name = self.TIMEOUTS.get(self.state)
        return bool(name and now - self.state_since >
                    getattr(self.config, name))

    def update(self, now: float, position=(0.0, 0.0, 0.0),
               px4_ok=True, failsafe=False, observation=None,
               intercept_reached=False, payload_result=None,
               landed=False, offboard_lost=False):
        """Advance one non-blocking state-machine tick."""
        if self.state in (MissionState.INITIALIZING,
                          MissionState.WAIT_FOR_START,
                          MissionState.COMPLETE):
            return
        if self.state == MissionState.EMERGENCY_LAND and landed:
            self.transition(MissionState.COMPLETE, now, 'emergency_landed')
            return
        if failsafe:
            self.transition(MissionState.EMERGENCY_LAND, now, 'px4_failsafe')
            return
        if (offboard_lost and self.state not in (
                MissionState.ABORT_RETURN, MissionState.EMERGENCY_LAND,
                MissionState.LAND)):
            self.transition(
                MissionState.ABORT_RETURN, now, 'offboard_unexpected_exit')
            return
        if not px4_ok:
            destination = (MissionState.ABORT_RETURN if self.home is not None
                           else MissionState.EMERGENCY_LAND)
            self.transition(destination, now, 'px4_or_localization_lost')
            return
        if self._elapsed(now) > self.config.mission_timeout_s:
            self.transition(MissionState.ABORT_RETURN, now, 'mission_timeout')
            return
        if self._timed_out(now):
            if self.state in (MissionState.RETURN_HOME,
                              MissionState.ABORT_RETURN,
                              MissionState.LAND):
                self.transition(
                    MissionState.EMERGENCY_LAND, now, 'state_timeout')
            else:
                self.transition(MissionState.ABORT_RETURN, now, 'state_timeout')
            return

        altitude = self.home[2] - position[2]
        if self.target_valid(observation, now):
            self.last_valid_observation = observation
        height_ok = abs(
            altitude - self.config.takeoff_height_m
        ) <= self.config.takeoff_height_tolerance_m
        if self.state == MissionState.TAKEOFF and height_ok:
            self.transition(MissionState.HOVER_STABLE, now, 'height_reached')
            self.hover_since = now
        elif self.state == MissionState.HOVER_STABLE:
            if not height_ok:
                self.hover_since = None
                self.transition(MissionState.TAKEOFF, now, 'height_left_band')
            elif now - self.hover_since >= self.config.stable_hover_duration_s:
                self.transition(
                    MissionState.TRANSIT_TO_INTERCEPT, now, 'hover_stable')
        elif (self.state == MissionState.TRANSIT_TO_INTERCEPT and
              intercept_reached):
            self.transition(MissionState.SEARCH_CAR, now, 'intercept_reached')
        elif self.state in (MissionState.SEARCH_CAR,
                            MissionState.ACQUIRE_CAR):
            if self.target_valid(observation, now):
                self.confirm_frames += 1
                if self.state == MissionState.SEARCH_CAR:
                    self.transition(
                        MissionState.ACQUIRE_CAR, now, 'target_candidate')
                if self.confirm_frames >= self.config.target_confirm_frames:
                    self.transition(
                        MissionState.FOLLOW_CAR, now, 'target_confirmed')
                    self.follow_since = now
                    self.target_lost_since = None
            else:
                self.confirm_frames = 0
        elif self.state == MissionState.FOLLOW_CAR:
            if not self.target_valid(observation, now):
                if self.target_lost_since is None:
                    self.target_lost_since = now
                if now - self.target_lost_since >= self.config.target_long_loss_s:
                    self.transition(
                        MissionState.SEARCH_CAR, now, 'target_long_loss')
            else:
                self.target_lost_since = None
                aligned = (abs(observation.error_x) <=
                           self.config.drop_alignment_tolerance and
                           abs(observation.error_y) <=
                           self.config.drop_alignment_tolerance)
                if aligned and now - self.follow_since >= \
                        self.config.follow_stable_duration_s:
                    self.transition(
                        MissionState.DROP_ALIGN, now, 'follow_stable')
        elif self.state == MissionState.DROP_ALIGN:
            aligned = (self.target_valid(observation, now) and
                       abs(observation.error_x) <=
                       self.config.drop_alignment_tolerance and
                       abs(observation.error_y) <=
                       self.config.drop_alignment_tolerance)
            if aligned and not self.payload_requested:
                self.payload_requested = True
                self.transition(
                    MissionState.RELEASE_PAYLOAD, now, 'drop_interlocks_ok')
        elif (self.state == MissionState.RELEASE_PAYLOAD and
              payload_result in ('SUCCESS', 'FAILED', 'REJECTED', 'TIMEOUT')):
            self.transition(
                MissionState.RETURN_HOME, now,
                'payload_%s' % payload_result.lower())
        elif self.state in (MissionState.RETURN_HOME,
                            MissionState.ABORT_RETURN):
            home_above = (
                self.home[0], self.home[1],
                self.home[2] - self.config.takeoff_height_m)
            distance = math.dist(position, home_above)
            if distance <= self.config.return_home_tolerance_m:
                if self.home_since is None:
                    self.home_since = now
                elif now - self.home_since >= \
                        self.config.home_stable_duration_s:
                    self.transition(MissionState.LAND, now, 'home_stable')
            else:
                self.home_since = None
        elif self.state == MissionState.LAND and landed:
            self.transition(MissionState.COMPLETE, now, 'landed')

    def command(self, now: float):
        """Return a high-level command, never a PX4 message."""
        if self.state in (MissionState.TAKEOFF, MissionState.HOVER_STABLE):
            return {'mode': 'POSITION',
                    'target': (self.home[0], self.home[1],
                               self.home[2] - self.config.takeoff_height_m)}
        if self.state == MissionState.TRANSIT_TO_INTERCEPT:
            field_x = self.config.intercept_x_m + self.predicted_distance(now)
            local_x, local_y = self.field_to_local(
                field_x, self.config.intercept_y_m)
            return {'mode': 'POSITION',
                    'target': (self.home[0] + local_x,
                               self.home[1] + local_y,
                               self.home[2] - self.config.takeoff_height_m)}
        if (self.state in (MissionState.ACQUIRE_CAR,
                           MissionState.FOLLOW_CAR,
                           MissionState.DROP_ALIGN) and
                self.target_valid(self.last_valid_observation, now)):
            return {
                'mode': 'VISION',
                'target': (
                    self.last_valid_observation.error_x,
                    self.last_valid_observation.error_y,
                    self.home[2] - self.config.takeoff_height_m),
            }
        if self.state in (MissionState.RETURN_HOME,
                          MissionState.ABORT_RETURN):
            return {'mode': 'POSITION',
                    'target': (self.home[0], self.home[1],
                               self.home[2] - self.config.takeoff_height_m)}
        if self.state in (MissionState.LAND, MissionState.EMERGENCY_LAND):
            return {'mode': 'LAND', 'target': self.home}
        return {'mode': 'HOLD', 'target': None}
