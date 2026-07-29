"""ROS-independent safety and state machine for all D-task mission modes."""

import math

from .mission_schema import CarProgress, parse_mission_mode


class MissionLogic:
    def __init__(self, mission_mode='hover_test', target_altitude=1.5,
                 simulation_mode=False, competition_mode=False,
                 enable_control=False, enable_auto_arm=False,
                 enable_visual_follow=True, enable_payload_release=False,
                 enable_dynamic_landing=False, enable_second_takeoff=False,
                 prestream_seconds=1.0, hover_confirm_seconds=3.0,
                 hover_test_seconds=10.0, dwell_on_car_seconds=5.0,
                 mission_timeout_seconds=90.0, b_deadline_seconds=15.0,
                 altitude_tolerance=0.1, stable_seconds=1.0,
                 payload_ack_timeout=3.0, touchdown_verify_seconds=1.0):
        self.mode = parse_mission_mode(mission_mode)
        self.mode_name = mission_mode
        self.target_altitude = float(target_altitude)
        self.simulation_mode = bool(simulation_mode)
        self.competition_mode = bool(competition_mode)
        self.enable_control = bool(enable_control)
        self.enable_auto_arm = bool(enable_auto_arm)
        self.enable_visual_follow = bool(enable_visual_follow)
        self.enable_payload_release = bool(enable_payload_release)
        self.enable_dynamic_landing = bool(enable_dynamic_landing)
        self.enable_second_takeoff = bool(enable_second_takeoff)
        self.prestream_seconds = float(prestream_seconds)
        self.hover_confirm_seconds = float(hover_confirm_seconds)
        self.hover_test_seconds = float(hover_test_seconds)
        self.dwell_on_car_seconds = max(5.0, float(dwell_on_car_seconds))
        self.mission_timeout_seconds = float(mission_timeout_seconds)
        self.b_deadline_seconds = float(b_deadline_seconds)
        self.altitude_tolerance = float(altitude_tolerance)
        self.stable_seconds = float(stable_seconds)
        self.payload_ack_timeout = float(payload_ack_timeout)
        self.touchdown_verify_seconds = max(
            0.0, float(touchdown_verify_seconds))
        self.reset()

    def reset(self):
        self.state = 'WAIT_PX4'
        self.state_since = 0.0
        self.started_at = None
        self.h = None
        self.position = None
        self.velocity = (0.0, 0.0, 0.0)
        self.heading = 0.0
        self.px4_fresh = False
        self.attitude_valid = False
        self.abnormal_tilt = False
        self.failsafe = False
        self.armed = False
        self.offboard = False
        self.ever_offboard = False
        self.start_signal = False
        self.safety_ready = False
        self.car_progress = CarProgress.UNKNOWN_OR_IDLE
        self.car_regression = False
        self.target_ok = False
        self.aligned = False
        self.touchdown = False
        self.payload_sent = False
        self.payload_ack = False
        self.second_cycle = False
        self.second_takeoff_origin = None
        self.event = 'RESET'
        self.stable_since = None

    def transition(self, state, now, event=None):
        self.state = state
        self.state_since = float(now)
        self.stable_since = None
        if event:
            self.event = event

    @property
    def cruise_z(self):
        return None if self.h is None else self.h[2] - self.target_altitude

    def update_position(self, x, y, z, vx, vy, vz, heading, valid):
        self.px4_fresh = bool(valid) and all(math.isfinite(v)
                                             for v in (x, y, z, vx, vy, vz, heading))
        if self.px4_fresh:
            self.position = (float(x), float(y), float(z))
            self.velocity = (float(vx), float(vy), float(vz))
            self.heading = float(heading)
            if self.h is None and math.hypot(vx, vy) < 0.15 and abs(vz) < 0.15:
                self.h = (float(x), float(y), float(z), float(heading))

    def update_status(self, armed, offboard, failsafe):
        self.armed, self.offboard, self.failsafe = bool(armed), bool(offboard), bool(failsafe)
        if self.offboard:
            self.ever_offboard = True
        elif (self.ever_offboard and self.state not in
              ('TOUCHDOWN_VERIFY', 'DISARM_ON_CAR', 'DWELL_ON_CAR',
               'SECOND_PRESTREAM', 'REQUEST_OFFBOARD', 'SECOND_ARM',
               'WAIT_DISARM', 'COMPLETE', 'LAND_H', 'FAILSAFE_LAND')):
            self.state = 'EXTERNAL_CONTROL'
            self.event = 'PX4_EXITED_OFFBOARD'

    def update_car_progress(self, value):
        value = CarProgress(int(value))
        if value < self.car_progress:
            self.car_regression = True
            self.event = 'CAR_PROGRESS_REGRESSION'
            return False
        self.car_progress = value
        return True

    def auto_arm_allowed(self):
        base = all((self.enable_control, self.enable_auto_arm,
                    self.start_signal, self.px4_fresh, self.h is not None,
                    not self.failsafe, not self.abnormal_tilt))
        if self.simulation_mode:
            return base
        return base and self.competition_mode and self.safety_ready and self.attitude_valid

    def altitude_reached(self):
        return self.at_cruise_altitude() and math.sqrt(
            sum(
                v *
                v for v in self.velocity)) < 0.25

    def at_cruise_altitude(self):
        return (self.position is not None and
                abs(self.position[2] - self.cruise_z) <=
                self.altitude_tolerance)

    def stable(self, condition, now, seconds):
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = float(now)
        return now - self.stable_since >= seconds

    def step(self, now):
        now = float(now)
        if self.state in ('EXTERNAL_CONTROL', 'COMPLETE'):
            return
        if self.started_at is not None and now - self.started_at > self.mission_timeout_seconds:
            self.transition('FAILSAFE_LAND', now, 'MISSION_TIMEOUT')
        if (self.started_at is not None and not self.px4_fresh and
                self.state not in ('COMPLETE', 'EXTERNAL_CONTROL',
                                   'FAILSAFE_LAND')):
            self.transition('FAILSAFE_LAND', now, 'PX4_DATA_TIMEOUT')
        if self.failsafe or self.abnormal_tilt or self.car_regression:
            self.transition('FAILSAFE_LAND', now, 'SAFETY_FAULT')
        if self.state == 'FAILSAFE_LAND':
            return
        missed_b = (self.started_at is not None and
                    now - self.started_at > self.b_deadline_seconds and
                    self.car_progress < CarProgress.PASSED_B)
        if missed_b:
            self.event = 'B_DEADLINE_RISK'
        if self.car_progress >= CarProgress.PASSED_D and self.state in (
                'SEARCH_TARGET', 'FOLLOW_TARGET', 'DROP_ALIGN',
                'DROP_RELEASE', 'LANDING_ALIGN', 'DESCEND_ON_CAR'):
            self.transition('ABORT_RETURN_H', now, 'D_PASSED_ABORT')

        if self.state == 'WAIT_PX4' and self.px4_fresh and self.h is not None:
            self.transition('WAIT_SAFETY', now)
        elif self.state == 'WAIT_SAFETY':
            safe = self.simulation_mode or (self.safety_ready and self.attitude_valid)
            if safe:
                self.transition('WAIT_START', now)
        elif self.state == 'WAIT_START' and self.start_signal:
            self.started_at = now
            self.transition('PRESTREAM', now, 'MISSION_STARTED')
        elif (self.state in ('PRESTREAM', 'SECOND_PRESTREAM') and
              now - self.state_since >= self.prestream_seconds):
            self.transition('REQUEST_OFFBOARD', now)
        elif self.state == 'REQUEST_OFFBOARD' and self.offboard:
            if self.armed:
                target = 'SECOND_TAKEOFF' if self.second_cycle else 'TAKEOFF'
                self.transition(target, now)
            elif self.auto_arm_allowed():
                self.transition('SECOND_ARM' if self.second_cycle else 'ARMING', now)
            else:
                self.transition('WAIT_MANUAL_ARM', now)
        elif (self.state in ('ARMING', 'WAIT_MANUAL_ARM', 'SECOND_ARM') and
              self.armed and self.offboard):
            self.transition('SECOND_TAKEOFF' if self.state == 'SECOND_ARM' else 'TAKEOFF', now)
        elif (self.state in ('TAKEOFF', 'SECOND_TAKEOFF') and
              self.stable(self.altitude_reached(), now,
                          self.stable_seconds)):
            if self.state == 'SECOND_TAKEOFF':
                self.transition('RETURN_H', now)
            elif self.mode_name == 'hover_test':
                self.transition('HOVER_TEST', now)
            else:
                self.transition('HOVER_CONFIRM', now)
        elif (self.state == 'HOVER_CONFIRM' and
              now - self.state_since >= self.hover_confirm_seconds):
            self.transition('SEARCH_TARGET', now)
        elif self.state == 'HOVER_TEST' and now - self.state_since >= self.hover_test_seconds:
            self.transition('LAND_H', now)
        elif self.state == 'SEARCH_TARGET' and self.target_ok:
            self.transition('FOLLOW_TARGET', now)
        elif self.state == 'FOLLOW_TARGET' and self.aligned:
            self.transition('DROP_ALIGN' if self.mode_name == 'drop' else 'LANDING_ALIGN', now)
        elif (self.state == 'DROP_ALIGN' and self.aligned and
              self.at_cruise_altitude() and
              self.enable_payload_release and not self.payload_sent):
            self.payload_sent = True
            self.transition('DROP_RELEASE', now, 'PAYLOAD_RELEASE')
        elif self.state == 'DROP_RELEASE' and self.payload_ack:
            self.transition('RETURN_H', now, 'PAYLOAD_ACK')
        elif (self.state == 'DROP_RELEASE' and
              now - self.state_since >= self.payload_ack_timeout):
            self.transition('ABORT_RETURN_H', now, 'PAYLOAD_ACK_TIMEOUT')
        elif self.state == 'LANDING_ALIGN' and self.aligned and self.enable_dynamic_landing:
            self.transition('DESCEND_ON_CAR', now)
        elif self.state == 'DESCEND_ON_CAR' and self.touchdown:
            self.transition('TOUCHDOWN_VERIFY', now)
        elif (self.state == 'TOUCHDOWN_VERIFY' and self.touchdown and
              now - self.state_since >= self.touchdown_verify_seconds):
            self.transition('DISARM_ON_CAR', now)
        elif self.state == 'DISARM_ON_CAR' and not self.armed:
            if self.position is not None:
                self.second_takeoff_origin = (
                    self.position[0], self.position[1], self.position[2])
            self.transition('DWELL_ON_CAR', now)
        elif self.state == 'DWELL_ON_CAR' and now - self.state_since >= self.dwell_on_car_seconds:
            if self.enable_second_takeoff:
                self.second_cycle = True
                self.transition('SECOND_PRESTREAM', now)
            else:
                self.transition('COMPLETE', now)
        elif self.state == 'ABORT_RETURN_H':
            self.transition('RETURN_H' if self.px4_fresh else 'FAILSAFE_LAND', now)
        elif self.state == 'RETURN_H' and self.at_h():
            self.transition('LAND_H', now)
        elif self.state == 'LAND_H' and not self.armed:
            self.transition('WAIT_DISARM', now)
        elif self.state == 'WAIT_DISARM' and not self.armed:
            self.transition('COMPLETE', now, 'MISSION_COMPLETE')

    def at_h(self):
        if self.h is None or self.position is None:
            return False
        distance = math.hypot(self.position[0] - self.h[0], self.position[1] - self.h[1])
        return distance <= 0.25 and abs(
            self.position[2] -
            self.cruise_z) <= self.altitude_tolerance and math.sqrt(
            sum(
                v *
                v for v in self.velocity)) <= 0.25

    def reset_if_safe(self):
        if not self.armed and self.state in ('COMPLETE', 'WAIT_PX4', 'WAIT_START'):
            self.reset()
            return True
        return False
