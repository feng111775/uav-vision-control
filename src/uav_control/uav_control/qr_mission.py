"""Event-driven QR shelf mission state machine shared by SITL and hardware."""

from dataclasses import dataclass
import math


@dataclass
class MissionOutput:
    """One control-cycle task decision."""

    state: str
    use_vision: bool = False
    forward: float = 0.0
    left: float = 0.0
    yaw_rate: float = 0.0
    request_land: bool = False
    request_disarm: bool = False


class QRMission:
    """Advance only from timestamped sensor and PX4 events."""

    WAITING = 'WAITING'
    PRESTREAM = 'PRESTREAM'
    TAKEOFF = 'TAKEOFF'
    QR_SEARCH = 'QR_SEARCH'
    QR_INVENTORY = 'QR_INVENTORY'
    TARGET_ACQUIRE = 'TARGET_ACQUIRE'
    TARGET_APPROACH = 'TARGET_APPROACH'
    LASER_ALIGN = 'LASER_ALIGN'
    LASER_CONFIRM = 'LASER_CONFIRM'
    TRANSIT_TO_LANDING = 'TRANSIT_TO_LANDING'
    DOWN_ACQUIRE = 'DOWN_ACQUIRE'
    ALIGN = 'ALIGN'
    LAND = 'LAND'
    DISARM = 'DISARM'
    FAILSAFE = 'FAILSAFE'
    STATES = (WAITING, PRESTREAM, TAKEOFF, QR_SEARCH, QR_INVENTORY,
              TARGET_ACQUIRE, TARGET_APPROACH, LASER_ALIGN, LASER_CONFIRM,
              TRANSIT_TO_LANDING, DOWN_ACQUIRE, ALIGN, LAND, DISARM,
              FAILSAFE)

    def __init__(self, target_qr_id=7, event_timeout=1.0,
                 state_timeout=30.0, mission_timeout=180.0,
                 approach_area=15000.0, align_error=12.0,
                 transit_seconds=4.0, transit_speed=-0.20,
                 search_yaw_rate=0.2):
        numeric = (event_timeout, state_timeout, mission_timeout,
                   approach_area, align_error, transit_seconds,
                   transit_speed, search_yaw_rate)
        if target_qr_id not in range(1, 25):
            raise ValueError('target_qr_id must be 1..24')
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError('mission parameters must be finite')
        if min(event_timeout, state_timeout, mission_timeout,
               approach_area, align_error, transit_seconds) <= 0:
            raise ValueError('mission thresholds must be positive')
        self.target_qr_id = int(target_qr_id)
        self.event_timeout = float(event_timeout)
        self.state_timeout = float(state_timeout)
        self.mission_timeout = float(mission_timeout)
        self.approach_area = float(approach_area)
        self.align_error = float(align_error)
        self.transit_seconds = float(transit_seconds)
        self.transit_speed = float(transit_speed)
        self.search_yaw_rate = abs(float(search_yaw_rate))
        self.state = self.WAITING
        self.state_since = None
        self.started = None
        self.reason = ''
        self.transitions = []
        self.inventory_complete = False
        self.target_seen = False
        self.target_area = 0.0
        self.target_error = math.inf
        self.qr_time = None
        self.laser_aligned = False
        self.laser_time = None
        self.selected_camera = ''
        self.camera_time = None
        self.down_error = math.inf
        self.down_valid = False
        self.down_time = None
        self.landed = False
        self.land_time = None
        self.armed = False

    def _enter(self, target, now, reason):
        if target == self.state:
            return
        self.transitions.append({
            'from': self.state, 'to': target,
            'time': float(now), 'reason': reason})
        self.state = target
        self.state_since = float(now)
        self.reason = reason

    def start(self, now):
        """Begin only after the flight controller enters prestream."""
        if self.state == self.WAITING:
            self.started = float(now)
            self._enter(self.PRESTREAM, now, 'offboard prestream started')

    def update_inventory(self, complete, target_seen, area, error, stamp):
        values = (area, error, stamp)
        if not all(math.isfinite(float(value)) for value in values):
            return
        stamp = float(stamp)
        if self.qr_time is not None and stamp < self.qr_time:
            return
        self.inventory_complete = bool(complete)
        self.target_seen = bool(target_seen)
        self.target_area = max(0.0, float(area))
        self.target_error = abs(float(error))
        self.qr_time = stamp

    def update_laser(self, aligned, stamp):
        if not math.isfinite(float(stamp)):
            return
        if self.laser_time is not None and stamp < self.laser_time:
            return
        self.laser_aligned = bool(aligned)
        self.laser_time = float(stamp)

    def update_camera(self, camera, stamp):
        if camera not in ('front', 'down') or not math.isfinite(float(stamp)):
            return
        if self.camera_time is not None and stamp < self.camera_time:
            return
        self.selected_camera = camera
        self.camera_time = float(stamp)

    def update_down(self, valid, error, stamp):
        if not math.isfinite(float(error)) or not math.isfinite(float(stamp)):
            return
        if self.down_time is not None and stamp < self.down_time:
            return
        self.down_valid = bool(valid)
        self.down_error = abs(float(error))
        self.down_time = float(stamp)

    def update_flight(self, armed, landed, stamp):
        if not math.isfinite(float(stamp)):
            return
        self.armed = bool(armed)
        self.landed = bool(landed)
        self.land_time = float(stamp)

    def fresh(self, stamp, now):
        return stamp is not None and 0.0 <= float(now) - stamp <= \
            self.event_timeout

    def fail(self, now, reason):
        self._enter(self.FAILSAFE, now, reason)

    def step(self, now, flight_phase):
        """Advance at most one transition using actual event state."""
        now = float(now)
        if self.state == self.FAILSAFE:
            return MissionOutput(self.state)
        if self.state == self.DISARM:
            return MissionOutput(
                self.state, request_disarm=self.armed)
        if self.started is not None and now - self.started > \
                self.mission_timeout:
            self.fail(now, 'mission timeout')
            return MissionOutput(self.state)
        if self.state_since is not None and self.state not in (
                self.PRESTREAM, self.TAKEOFF, self.LAND) and \
                now - self.state_since > self.state_timeout:
            self.fail(now, '%s timeout' % self.state)
            return MissionOutput(self.state)

        if self.state == self.PRESTREAM and flight_phase == 'TAKEOFF':
            self._enter(self.TAKEOFF, now, 'PX4 armed and offboard')
        elif self.state == self.TAKEOFF and flight_phase == 'VISION_CONTROL':
            self._enter(self.QR_SEARCH, now, 'takeoff altitude reached')
        elif self.state == self.QR_SEARCH and self.fresh(self.qr_time, now):
            self._enter(self.QR_INVENTORY, now, 'fresh QR observation')
        elif self.state == self.QR_INVENTORY and self.inventory_complete:
            self._enter(self.TARGET_ACQUIRE, now, 'inventory condition met')
        elif self.state == self.TARGET_ACQUIRE and self.target_seen and \
                self.fresh(self.qr_time, now):
            self._enter(self.TARGET_APPROACH, now, 'target confirmed')
        elif self.state == self.TARGET_APPROACH:
            if not self.fresh(self.qr_time, now):
                self._enter(self.TARGET_ACQUIRE, now, 'target lost')
            elif self.target_area >= self.approach_area:
                self._enter(self.LASER_ALIGN, now, 'approach area reached')
        elif self.state == self.LASER_ALIGN and self.laser_aligned and \
                self.fresh(self.laser_time, now):
            self._enter(self.LASER_CONFIRM, now, 'laser frames confirmed')
        elif self.state == self.LASER_CONFIRM:
            self._enter(self.TRANSIT_TO_LANDING, now, 'software laser event')
        elif self.state == self.TRANSIT_TO_LANDING and \
                now - self.state_since >= self.transit_seconds:
            self._enter(self.DOWN_ACQUIRE, now, 'transit interval complete')
        elif self.state == self.DOWN_ACQUIRE and \
                self.selected_camera == 'down' and \
                self.fresh(self.camera_time, now):
            self._enter(self.ALIGN, now, 'down camera selected')
        elif self.state == self.ALIGN and self.down_valid and \
                self.down_error <= self.align_error and \
                self.fresh(self.down_time, now):
            self._enter(self.LAND, now, 'down target aligned')
        elif self.state == self.LAND and self.landed and \
                self.fresh(self.land_time, now):
            self._enter(self.DISARM, now, 'landing detector confirmed')

        if self.state in (self.QR_SEARCH, self.QR_INVENTORY,
                          self.TARGET_ACQUIRE):
            return MissionOutput(self.state, yaw_rate=self.search_yaw_rate)
        if self.state in (self.TARGET_APPROACH, self.LASER_ALIGN):
            return MissionOutput(self.state, use_vision=True)
        if self.state == self.TRANSIT_TO_LANDING:
            return MissionOutput(self.state, forward=self.transit_speed)
        if self.state == self.ALIGN:
            return MissionOutput(self.state, use_vision=True)
        if self.state == self.LAND:
            return MissionOutput(self.state, request_land=True)
        if self.state == self.DISARM and self.armed:
            return MissionOutput(self.state, request_disarm=True)
        return MissionOutput(self.state)
