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
    scan_position: tuple = ()
    vertical: float = math.nan


class QRMission:
    """Advance only from timestamped sensor and PX4 events."""

    WAITING = 'WAITING'
    PRESTREAM = 'PRESTREAM'
    TAKEOFF = 'TAKEOFF'
    QR_SCAN_MOVE = 'QR_SCAN_MOVE'
    QR_SCAN_HOLD = 'QR_SCAN_HOLD'
    QR_SCAN_CONFIRM = 'QR_SCAN_CONFIRM'
    QR_SCAN_NEXT = 'QR_SCAN_NEXT'
    QR_INVENTORY_COMPLETE = 'QR_INVENTORY_COMPLETE'
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
    STATES = (WAITING, PRESTREAM, TAKEOFF, QR_SCAN_MOVE, QR_SCAN_HOLD,
              QR_SCAN_CONFIRM, QR_SCAN_NEXT, QR_INVENTORY_COMPLETE,
              TARGET_ACQUIRE, TARGET_APPROACH, LASER_ALIGN, LASER_CONFIRM,
              TRANSIT_TO_LANDING, DOWN_ACQUIRE, ALIGN, LAND, DISARM,
              FAILSAFE)

    def __init__(self, target_qr_id=7, event_timeout=1.0,
                 state_timeout=30.0, mission_timeout=180.0,
                 approach_area=15000.0, align_error=12.0,
                 transit_seconds=4.0, transit_speed=-0.20,
                 search_yaw_rate=0.2, scan_hold_seconds=0.6,
                 scan_timeout=8.0, scan_max_retries=2):
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
        self.target_visible = False
        self.target_area = 0.0
        self.target_error = math.inf
        self.target_error_x = math.inf
        self.target_error_y = math.inf
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
        self.scan_order = (1, 2, 3, 4, 5, 6, 12, 11, 10, 9, 8, 7,
                           13, 14, 15, 16, 17, 18, 24, 23, 22, 21, 20, 19)
        self.scan_index = 0
        self.inventory_count = 0
        self.scan_hold_seconds = float(scan_hold_seconds)
        self.scan_timeout = float(scan_timeout)
        self.scan_max_retries = int(scan_max_retries)
        self.retry_count = 0
        self.position = (0.0, 0.0, 0.0)
        self.scan_arrived_time = None

    def scan_point(self):
        """Generate the current NED scan point from the 4x6 shelf layout."""
        qr_id = self.scan_order[min(self.scan_index, 23)]
        row, column = divmod(qr_id - 1, 6)
        # Shelf is at Gazebo x=1.5; hold 0.72 m in front of it.
        north = -1.25 + column * 0.5
        height = 2.50 - row * 0.5
        search = (0.12 if self.retry_count % 2 else -0.12) * (
            (self.retry_count + 1) // 2)
        # PX4 NED north maps Gazebo +Y and east maps Gazebo +X here.
        return (north + search, 0.78, -height)

    def target_point(self):
        row, column = divmod(self.target_qr_id - 1, 6)
        return (-1.25 + column * 0.5, 0.78, -(2.50 - row * 0.5))

    def update_position(self, north, east, down):
        self.position = (float(north), float(east), float(down))

    def at_scan_point(self, tolerance=0.12):
        return math.dist(self.position, self.scan_point()) <= tolerance

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

    def update_inventory(self, complete, target_seen, area, error, stamp,
                         scan_index=None, target_visible=None,
                         error_x=math.inf, error_y=math.inf):
        values = (area, stamp)
        if not all(math.isfinite(float(value)) for value in values):
            return
        if target_seen and not math.isfinite(float(error)):
            return
        stamp = float(stamp)
        if self.qr_time is not None and stamp < self.qr_time:
            return
        self.inventory_complete = bool(complete)
        self.target_seen = bool(target_seen)
        self.target_visible = bool(target_seen if target_visible is None
                                   else target_visible)
        self.target_area = max(0.0, float(area))
        self.target_error = abs(float(error))
        self.target_error_x = float(error_x)
        self.target_error_y = float(error_y)
        self.qr_time = stamp
        if scan_index is not None:
            self.inventory_count = max(
                self.inventory_count, min(24, int(scan_index)))

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
            self._enter(self.QR_SCAN_MOVE, now, 'takeoff altitude reached')
        elif self.state == self.QR_SCAN_MOVE and self.at_scan_point():
            self.scan_arrived_time = now
            self._enter(self.QR_SCAN_HOLD, now, 'scan point reached')
        elif self.state == self.QR_SCAN_HOLD:
            if not self.at_scan_point():
                self._enter(self.QR_SCAN_MOVE, now, 'scan hold drift')
            elif now - self.scan_arrived_time >= self.scan_hold_seconds:
                self._enter(self.QR_SCAN_CONFIRM, now, 'scan hold stable')
        elif self.state == self.QR_SCAN_CONFIRM:
            if self.inventory_count > self.scan_index:
                self.scan_index = self.inventory_count
                self.retry_count = 0
                if self.scan_index == 24:
                    self._enter(self.QR_INVENTORY_COMPLETE, now,
                                'all 24 scan points confirmed')
                else:
                    self._enter(self.QR_SCAN_NEXT, now,
                                'expected QR confirmed')
            elif now - self.state_since > self.scan_timeout:
                self.retry_count += 1
                if self.retry_count > self.scan_max_retries:
                    self.fail(now, 'QR %d scan timeout after %d retries' % (
                        self.scan_order[self.scan_index],
                        self.scan_max_retries))
                else:
                    self._enter(self.QR_SCAN_MOVE, now,
                                'bounded scan retry %d' % self.retry_count)
        elif self.state == self.QR_SCAN_NEXT:
            self._enter(self.QR_SCAN_MOVE, now, 'moving to next scan point')
        elif self.state == self.QR_INVENTORY_COMPLETE:
            if not self.inventory_complete or self.inventory_count != 24:
                self.fail(now, 'inventory completion mismatch')
            elif not self.target_seen:
                self.fail(now, 'target QR missing from completed inventory')
            else:
                self._enter(self.TARGET_ACQUIRE, now,
                            'inventory complete; target selected')
        elif False:
            self._enter(self.TARGET_ACQUIRE, now, 'inventory condition met')
        elif self.state == self.TARGET_ACQUIRE and self.target_visible and \
                math.dist(self.position, self.target_point()) <= 0.12 and \
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

        if self.state in (self.QR_SCAN_MOVE, self.QR_SCAN_HOLD,
                          self.QR_SCAN_CONFIRM, self.QR_SCAN_NEXT):
            return MissionOutput(self.state, scan_position=self.scan_point())
        if self.state == self.TARGET_ACQUIRE:
            return MissionOutput(self.state, scan_position=self.target_point())
        if self.state == self.TARGET_APPROACH:
            return MissionOutput(
                self.state, use_vision=True,
                scan_position=self.target_point())
        if self.state == self.LASER_ALIGN:
            left = max(-0.15, min(0.15, -0.002 * self.target_error_x))
            vertical = max(
                -0.30, min(0.30, 0.002 * self.target_error_y))
            return MissionOutput(
                self.state, left=left, vertical=vertical)
        if self.state == self.TRANSIT_TO_LANDING:
            return MissionOutput(self.state, forward=self.transit_speed)
        if self.state == self.ALIGN:
            return MissionOutput(self.state, use_vision=True)
        if self.state == self.LAND:
            return MissionOutput(self.state, request_land=True)
        if self.state == self.DISARM and self.armed:
            return MissionOutput(self.state, request_disarm=True)
        return MissionOutput(self.state)
