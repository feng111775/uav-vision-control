"""Pure, deterministic stage 4C mission logic."""

from dataclasses import dataclass
from enum import Enum
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile


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
    target_min_confidence: float = 0.6
    target_confirm_frames: int = 3
    target_short_loss_s: float = 0.5
    target_long_loss_s: float = 2.0
    follow_stable_duration_s: float = 2.0
    follow_stable_speed_mps: float = 0.05
    follow_error_gain: float = 0.3
    drop_alignment_tolerance: float = 0.08
    release_min_return_time_s: float = 30.0
    return_home_tolerance_m: float = 0.2
    home_stable_duration_s: float = 1.0
    takeoff_timeout_s: float = 15.0
    transit_timeout_s: float = 15.0
    search_timeout_s: float = 15.0
    search_speed_mps: float = 0.18
    search_max_speed_mps: float = 0.18
    search_max_distance_m: float = 3.0
    search_acceleration_mps2: float = 0.1
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

    def __post_init__(self):
        """Reject unsafe or contradictory numeric mission limits."""
        numeric = vars(self)
        if not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in numeric.values()):
            raise ValueError('all mission parameters must be finite numbers')
        if self.target_short_loss_s < 0.0 or self.target_long_loss_s < 0.0:
            raise ValueError('target loss thresholds cannot be negative')
        if self.target_short_loss_s > self.target_long_loss_s:
            raise ValueError(
                'target_short_loss_s cannot exceed target_long_loss_s')
        if self.takeoff_height_m <= 0.0:
            raise ValueError('takeoff_height_m must be positive')
        if self.mission_timeout_s <= 0.0:
            raise ValueError('mission_timeout_s must be positive')
        if not 0.0 <= self.target_min_confidence <= 1.0:
            raise ValueError('target_min_confidence must be in [0, 1]')
        if not 0.0 <= self.search_speed_mps <= self.search_max_speed_mps:
            raise ValueError('search_speed_mps exceeds its safe range')
        if not 0.0 < self.search_max_speed_mps <= 0.5:
            raise ValueError('search_max_speed_mps must be in (0, 0.5]')
        if self.search_max_distance_m <= 0.0:
            raise ValueError('search_max_distance_m must be positive')
        if self.search_acceleration_mps2 <= 0.0:
            raise ValueError('search_acceleration_mps2 must be positive')
        if self.follow_stable_speed_mps < 0.0:
            raise ValueError('follow_stable_speed_mps cannot be negative')
        if self.follow_error_gain < 0.0:
            raise ValueError('follow_error_gain cannot be negative')
        if not 0.0 <= self.release_min_return_time_s < self.mission_timeout_s:
            raise ValueError(
                'release_min_return_time_s must fit inside mission timeout')


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


def validate_control_interlocks(simulation_mode: bool, confirm_sitl_only: bool,
                                enable_control: bool, enable_auto_arm: bool,
                                hardware_bench_mode: bool):
    """Reject contradictory SITL and real-Pixhawk bench control settings."""
    if confirm_sitl_only and not simulation_mode:
        raise ValueError('confirm_sitl_only requires simulation_mode=true')
    if enable_control and simulation_mode and not confirm_sitl_only:
        raise ValueError('SITL control requires confirm_sitl_only=true')
    if enable_auto_arm and not enable_control:
        raise ValueError('automatic arming requires control to be enabled')
    if hardware_bench_mode and (
            simulation_mode or confirm_sitl_only or enable_control or
            enable_auto_arm):
        raise ValueError('hardware bench configuration must be fail-closed')
    return True


def resolve_takeoff_height(takeoff_height_m: float,
                           target_altitude: float) -> float:
    """Use relative-Home height while rejecting a conflicting legacy value."""
    values = (float(takeoff_height_m), float(target_altitude))
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError('flight height must be finite and positive')
    if not math.isclose(values[0], values[1], abs_tol=1e-9):
        raise ValueError(
            'deprecated target_altitude conflicts with takeoff_height_m')
    return values[0]


class HorizontalVelocityLimiter:
    """Deterministic 2-D acceleration and magnitude limiter."""

    def __init__(self, acceleration_mps2: float, max_speed_mps: float):
        values = (float(acceleration_mps2), float(max_speed_mps))
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('velocity limits must be finite and positive')
        self.acceleration = values[0]
        self.max_speed = values[1]
        self.value = (0.0, 0.0)

    def update(self, target, dt: float):
        """Move toward target without exceeding acceleration or speed."""
        values = (float(target[0]), float(target[1]), float(dt))
        if not all(math.isfinite(value) for value in values) or values[2] <= 0:
            raise ValueError('velocity target and dt must be finite')
        dx = values[0] - self.value[0]
        dy = values[1] - self.value[1]
        delta = math.hypot(dx, dy)
        step = self.acceleration * values[2]
        scale = 1.0 if delta <= step else step / delta
        result = (
            self.value[0] + dx * scale,
            self.value[1] + dy * scale,
        )
        magnitude = math.hypot(*result)
        if magnitude > self.max_speed:
            result = (
                result[0] * self.max_speed / magnitude,
                result[1] * self.max_speed / magnitude,
            )
        self.value = result
        return result


class StartGate:
    """Accept each task identifier once, without triggering flight itself."""

    def __init__(self, state_path: str = ''):
        self.accepted = set()
        self.closed = False
        self.last_counter = {}
        self.state_path = Path(state_path) if state_path else None
        if self.state_path and self.state_path.exists():
            try:
                data = json.loads(self.state_path.read_text())
                self.accepted = {
                    tuple(item) for item in data.get('accepted', [])}
                self.last_counter = {
                    str(key): int(value) for key, value in
                    data.get('last_counter', {}).items()}
            except (OSError, TypeError, ValueError, json.JSONDecodeError):
                self.closed = True

    def _persist(self):
        if not self.state_path:
            return
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=self.state_path.name + '.',
            dir=str(self.state_path.parent))
        try:
            with os.fdopen(descriptor, 'w') as stream:
                json.dump(
                    {
                        'accepted': [
                            list(item) for item in sorted(self.accepted)],
                        'last_counter': self.last_counter,
                    },
                    stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.state_path)
        except Exception:
            try:
                os.unlink(temporary)
            except OSError:
                pass
            self.closed = True
            raise

    def accept(self, task_id: str, valid: bool) -> bool:
        """Accept a new valid identifier unless the gate is closed."""
        if self.closed or not valid or not task_id or task_id in self.accepted:
            return False
        self.accepted.add(task_id)
        return True

    def accept_protocol(self, session_id: str, start_id: str, valid: bool,
                        ready: bool, busy: bool,
                        sender_counter: int | None = None) -> str:
        """Classify an idempotent session/start request without flight access."""
        key = (str(session_id), str(start_id))
        if key in self.accepted:
            return 'ALREADY_PROCESSED'
        if not valid or not all(key):
            return 'START_REJECTED'
        if sender_counter is not None:
            counter = int(sender_counter)
            if counter <= self.last_counter.get(key[0], -1):
                return 'START_REJECTED'
        if not ready:
            return 'NOT_READY'
        if busy or self.closed:
            return 'BUSY'
        self.accepted.add(key)
        if sender_counter is not None:
            self.last_counter[key[0]] = int(sender_counter)
        self._persist()
        return 'START_ACCEPTED'


class StartProtocol:
    """Transport-neutral versioned CAR_START parser and checksum validator."""

    VERSION = 1
    COMMANDS = {'CAR_START', 'ABORT', 'HEARTBEAT'}

    @staticmethod
    def checksum(fields: dict) -> str:
        """Return SHA-256 over canonical protocol fields."""
        canonical = json.dumps(
            fields, sort_keys=True, separators=(',', ':')).encode()
        return hashlib.sha256(canonical).hexdigest()

    @classmethod
    def build(cls, session_id: str, start_id: str, sender_counter: int,
              sender_timestamp_ns: int, command='CAR_START', **optional):
        """Build a deterministic simulation/test packet."""
        fields = {
            'protocol_version': cls.VERSION,
            'session_id': str(session_id),
            'start_id': str(start_id),
            'sender_counter': int(sender_counter),
            'sender_timestamp_ns': int(sender_timestamp_ns),
            'command': str(command),
        }
        fields.update(optional)
        return {**fields, 'checksum': cls.checksum(fields)}

    @classmethod
    def parse(cls, packet: dict, now_ns: int, max_age_s: float):
        """Validate version, fields, checksum and sender timestamp freshness."""
        required = {
            'protocol_version', 'session_id', 'start_id', 'sender_counter',
            'sender_timestamp_ns', 'command', 'checksum'}
        if not isinstance(packet, dict) or not required <= set(packet):
            raise ValueError('missing protocol fields')
        checksum = str(packet['checksum'])
        fields = {key: value for key, value in packet.items()
                  if key != 'checksum'}
        if not hashlib.sha256(
                json.dumps(fields, sort_keys=True, separators=(',', ':')
                           ).encode()).hexdigest() == checksum:
            raise ValueError('checksum mismatch')
        if int(fields['protocol_version']) != cls.VERSION:
            raise ValueError('unsupported protocol version')
        if str(fields['command']) not in cls.COMMANDS:
            raise ValueError('unsupported command')
        if not str(fields['session_id']) or not str(fields['start_id']):
            raise ValueError('empty identifier')
        if int(fields['sender_counter']) < 0:
            raise ValueError('negative sender counter')
        age_ns = int(now_ns) - int(fields['sender_timestamp_ns'])
        if age_ns < 0 or age_ns > float(max_age_s) * 1e9:
            raise ValueError('stale or future packet')
        return fields


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


class PersistentPayloadGate:
    """Atomic, restart-safe software-only payload transaction journal."""

    LEDGER_FROM_STATE = {
        'IDLE': 'IDLE',
        'ACCEPTED': 'REQUEST_ACCEPTED',
        'EXECUTING': 'EXECUTION_STARTED',
        'DRY_RUN_CONFIRMED': 'DRY_RUN_CONFIRMED',
        'SUCCESS': 'DRY_RUN_CONFIRMED',
        'FAILED': 'FAILED_SAFE',
        'TIMEOUT': 'UNKNOWN_LOCKED',
        'REJECTED': 'FAILED_SAFE',
        'LOCKED': 'UNKNOWN_LOCKED',
    }
    STATE_FROM_LEDGER = {
        value: key for key, value in LEDGER_FROM_STATE.items()}
    STATE_FROM_LEDGER.update({
        'DRY_RUN_CONFIRMED': 'DRY_RUN_CONFIRMED',
        'PHYSICAL_CONFIRMED': 'LOCKED',
        'FAILED_SAFE': 'FAILED',
        'UNKNOWN_LOCKED': 'LOCKED',
    })
    TERMINAL = {
        'DRY_RUN_CONFIRMED', 'SUCCESS', 'FAILED', 'TIMEOUT', 'REJECTED'}
    STATES = {'IDLE', 'ACCEPTED', 'EXECUTING'} | TERMINAL | {'LOCKED'}

    def __init__(self, state_path: str):
        self.path = Path(state_path)
        self.state = 'IDLE'
        self.session_id = ''
        self.release_id = ''
        self.processed = set()
        self.outcomes = {}
        self.historical_execution_count = 0
        self._load()

    @property
    def ledger_state(self):
        """Return the unambiguous persistent actuator transaction state."""
        return self.LEDGER_FROM_STATE[self.state]

    def _load(self):
        if not self.path.exists():
            return
        try:
            data = json.loads(self.path.read_text())
            state = data.get('state')
            ledger_state = data.get('ledger_state')
            if ledger_state is not None:
                if ledger_state == 'EXECUTION_STARTED':
                    state = 'EXECUTING'
                else:
                    state = self.STATE_FROM_LEDGER.get(ledger_state)
            if state not in self.STATES:
                raise ValueError('unknown payload state')
            self.processed = {
                tuple(item) for item in data.get('processed', [])}
            self.outcomes = {
                tuple(key.split('\u001f', 1)): str(value)
                for key, value in data.get('outcomes', {}).items()}
            self.historical_execution_count = int(
                data.get('historical_execution_count', 0))
            if self.historical_execution_count < 0:
                raise ValueError('negative payload execution history')
            self.session_id = str(data.get('session_id', ''))
            self.release_id = str(data.get('release_id', ''))
            self.state = 'LOCKED' if state == 'EXECUTING' else state
        except (OSError, TypeError, ValueError, KeyError, json.JSONDecodeError):
            self.state = 'LOCKED'

    def _write(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {
            'state': self.state,
            'ledger_state': self.ledger_state,
            'session_id': self.session_id,
            'release_id': self.release_id,
            'processed': [list(item) for item in sorted(self.processed)],
            'outcomes': {
                '\u001f'.join(key): value
                for key, value in sorted(self.outcomes.items())},
            'historical_execution_count': self.historical_execution_count,
        }
        descriptor, temporary = tempfile.mkstemp(
            prefix=self.path.name + '.', dir=str(self.path.parent))
        try:
            with os.fdopen(descriptor, 'w') as stream:
                json.dump(data, stream, sort_keys=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        except Exception:
            try:
                os.unlink(temporary)
            except OSError:
                pass
            self.state = 'LOCKED'
            raise

    def begin(self, session_id: str, release_id: str, allowed: bool) -> str:
        """Atomically enter EXECUTING once for a valid software request."""
        key = (str(session_id), str(release_id))
        if self.state == 'LOCKED':
            return 'LOCKED'
        if key in self.processed:
            return 'REJECTED'
        if not allowed or not all(key):
            return 'REJECTED'
        self.session_id, self.release_id = key
        self.state = 'ACCEPTED'
        self._write()
        self.state = 'EXECUTING'
        self.historical_execution_count += 1
        self._write()
        return self.state

    def finish(self, result: str) -> str:
        """Persist one simulated terminal result."""
        result = str(result).upper()
        if self.state != 'EXECUTING' or result not in self.TERMINAL:
            return 'LOCKED' if self.state == 'LOCKED' else 'REJECTED'
        self.processed.add((self.session_id, self.release_id))
        self.outcomes[(self.session_id, self.release_id)] = result
        self.state = result
        self._write()
        return result

    def cancel(self) -> str:
        """Fail closed: cancellation ends software execution as REJECTED."""
        if self.state != 'EXECUTING':
            return 'LOCKED' if self.state == 'LOCKED' else 'REJECTED'
        self.processed.add((self.session_id, self.release_id))
        self.outcomes[(self.session_id, self.release_id)] = 'REJECTED'
        self.state = 'REJECTED'
        self._write()
        return self.state

    def reset_lock(self, explicit_confirmation: str) -> bool:
        """Require an explicit operator phrase to clear an uncertain journal."""
        if self.state != 'LOCKED' or explicit_confirmation != 'RESET_LOCK':
            return False
        self.state = 'IDLE'
        self.session_id = self.release_id = ''
        self._write()
        return True

    def previous_result(self, session_id: str, release_id: str):
        """Return a persisted terminal result without re-executing."""
        return self.outcomes.get((str(session_id), str(release_id)))


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
        self.follow_stable_since = None
        self.target_lost_since = None
        self.confirm_frames = 0
        self.payload_requested = False
        self.last_valid_observation = None
        self.search_heading = None
        self.search_origin = None
        self.search_started = None
        self.target_first_seen = None
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
                    observation.fresh(now, self.config.target_max_age_s) and
                    observation.confidence >=
                    self.config.target_min_confidence)

    def enter_search(self, now: float, position, heading, reason: str) -> bool:
        """Lock a finite local-NED heading and position at search entry."""
        values = tuple(position) + (heading,)
        if len(position) != 3 or not all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in values):
            self.last_reason = 'search_heading_or_position_not_ready'
            return False
        self.search_heading = float(heading)
        self.search_origin = tuple(float(value) for value in position)
        self.search_started = float(now)
        self.confirm_frames = 0
        self.target_first_seen = None
        self.transition(MissionState.SEARCH_CAR, now, reason)
        return True

    def search_velocity(self) -> tuple[float, float]:
        """Convert locked body-forward search speed to local NED velocity."""
        if self.search_heading is None:
            return 0.0, 0.0
        speed = self.config.search_speed_mps
        return (
            speed * math.cos(self.search_heading),
            speed * math.sin(self.search_heading),
        )

    def search_distance(self, position) -> float:
        """Return horizontal distance from the locked search origin."""
        if self.search_origin is None:
            return 0.0
        return math.hypot(
            float(position[0]) - self.search_origin[0],
            float(position[1]) - self.search_origin[1])

    def _elapsed(self, now: float) -> float:
        return 0.0 if self.task_started is None else now - self.task_started

    def _timed_out(self, now: float) -> bool:
        name = self.TIMEOUTS.get(self.state)
        return bool(name and now - self.state_since >
                    getattr(self.config, name))

    def update(self, now: float, position=(0.0, 0.0, 0.0),
               px4_ok=True, failsafe=False, observation=None,
               intercept_reached=False, payload_result=None,
               landed=False, offboard_lost=False, heading=None,
               communication_lost=False):
        """Advance one non-blocking state-machine tick."""
        if self.state in (MissionState.INITIALIZING,
                          MissionState.WAIT_FOR_START,
                          MissionState.COMPLETE):
            return
        if self.state == MissionState.EMERGENCY_LAND:
            if landed:
                self.transition(
                    MissionState.COMPLETE, now, 'emergency_landed')
            return
        if failsafe:
            self.transition(MissionState.EMERGENCY_LAND, now, 'px4_failsafe')
            return
        if (communication_lost and self.state not in (
                MissionState.ABORT_RETURN, MissionState.EMERGENCY_LAND,
                MissionState.LAND)):
            self.transition(
                MissionState.ABORT_RETURN, now, 'start_transport_lost')
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
        if (self._elapsed(now) > self.config.mission_timeout_s and
                self.state not in (
                    MissionState.ABORT_RETURN,
                    MissionState.EMERGENCY_LAND,
                    MissionState.LAND)):
            self.transition(MissionState.ABORT_RETURN, now, 'mission_timeout')
            return
        if (self.state in (MissionState.SEARCH_CAR,
                           MissionState.ACQUIRE_CAR) and
                (self.search_started is None or
                 now - self.search_started >=
                 self.config.search_timeout_s)):
            self.transition(
                MissionState.ABORT_RETURN, now, 'search_timeout')
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
            if not self.enter_search(
                    now, position, heading, 'intercept_reached'):
                self.transition(
                    MissionState.ABORT_RETURN, now, 'search_not_ready')
        elif self.state in (MissionState.SEARCH_CAR,
                            MissionState.ACQUIRE_CAR):
            if self.search_distance(position) >= \
                    self.config.search_max_distance_m:
                self.transition(
                    MissionState.ABORT_RETURN, now, 'search_distance_exceeded')
                return
            if self.target_valid(observation, now):
                if self.target_first_seen is None:
                    self.target_first_seen = now
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
                self.target_first_seen = None
                if self.state == MissionState.ACQUIRE_CAR:
                    self.transition(
                        MissionState.SEARCH_CAR, now, 'target_candidate_lost')
        elif self.state == MissionState.FOLLOW_CAR:
            if not self.target_valid(observation, now):
                if self.target_lost_since is None:
                    self.target_lost_since = now
                loss = max(0.0, now - self.target_lost_since)
                if loss >= self.config.target_long_loss_s:
                    if not self.enter_search(
                            now, position, heading, 'target_long_loss'):
                        self.transition(
                            MissionState.ABORT_RETURN, now, 'search_not_ready')
                elif loss >= self.config.target_short_loss_s:
                    self.last_reason = 'target_short_loss_hold'
            else:
                self.target_lost_since = None
                aligned = (abs(observation.error_x) <=
                           self.config.drop_alignment_tolerance and
                           abs(observation.error_y) <=
                           self.config.drop_alignment_tolerance)
                speed_estimate = self.config.follow_error_gain * math.hypot(
                    observation.error_x, observation.error_y)
                stable = (
                    aligned and
                    speed_estimate <= self.config.follow_stable_speed_mps)
                if stable and self.follow_stable_since is None:
                    self.follow_stable_since = now
                if not stable:
                    self.follow_stable_since = None
                if (self.follow_stable_since is not None and
                        now - self.follow_stable_since >=
                        self.config.follow_stable_duration_s):
                    self.transition(
                        MissionState.DROP_ALIGN, now, 'follow_stable')
        elif self.state == MissionState.DROP_ALIGN:
            aligned = (self.target_valid(observation, now) and
                       abs(observation.error_x) <=
                       self.config.drop_alignment_tolerance and
                       abs(observation.error_y) <=
                       self.config.drop_alignment_tolerance)
            speed_ok = (
                observation is not None and
                self.config.follow_error_gain * math.hypot(
                    observation.error_x, observation.error_y) <=
                self.config.follow_stable_speed_mps)
            time_remaining = (
                self.config.mission_timeout_s - self._elapsed(now))
            if (aligned and speed_ok and height_ok and
                    time_remaining >=
                    self.config.release_min_return_time_s and
                    not self.payload_requested):
                self.payload_requested = True
                self.transition(
                    MissionState.RELEASE_PAYLOAD, now, 'drop_interlocks_ok')
        elif (self.state == MissionState.RELEASE_PAYLOAD and
              payload_result in (
                  'DRY_RUN_CONFIRMED', 'SUCCESS', 'FAILED', 'REJECTED',
                  'TIMEOUT')):
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
        if self.state in (MissionState.SEARCH_CAR,
                          MissionState.ACQUIRE_CAR):
            north, east = self.search_velocity()
            return {
                'mode': 'SEARCH',
                'velocity': (north, east),
                'target_z': self.home[2] - self.config.takeoff_height_m,
                'search_heading': self.search_heading,
            }
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
                'confidence': self.last_valid_observation.confidence,
                'observation_stamp': self.last_valid_observation.stamp,
            }
        if self.state == MissionState.FOLLOW_CAR:
            return {
                'mode': 'HOLD',
                'target_z': self.home[2] - self.config.takeoff_height_m,
            }
        if self.state in (MissionState.RETURN_HOME,
                          MissionState.ABORT_RETURN):
            return {'mode': 'POSITION',
                    'target': (self.home[0], self.home[1],
                               self.home[2] - self.config.takeoff_height_m)}
        if self.state in (MissionState.LAND, MissionState.EMERGENCY_LAND):
            return {'mode': 'LAND', 'target': self.home}
        return {'mode': 'HOLD', 'target': None}
