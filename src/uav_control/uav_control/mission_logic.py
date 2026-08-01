"""ROS-independent unified state machine for all D-task mission modes."""

import math

from .mission_schema import CarProgress, MissionState, parse_mission_mode


S = MissionState


class OdomFrameGate:
    """Require consecutive, fresh PX4 odometry samples before flight."""

    def __init__(self, min_frames=20, min_source_span_seconds=1.0,
                 timeout_seconds=0.5):
        self.min_frames = int(min_frames)
        self.min_source_span_us = int(
            float(min_source_span_seconds) * 1_000_000)
        self.timeout_seconds = float(timeout_seconds)
        if self.min_frames < 20:
            raise ValueError('odom gate requires at least 20 new frames')
        if self.min_source_span_us < 1_000_000:
            raise ValueError('odom gate source span must be at least 1 second')
        if self.timeout_seconds <= 0.0:
            raise ValueError('odom gate timeout must be positive')
        self.reset()

    def reset(self):
        """Discard the complete consecutive-frame window."""
        self.consecutive_frames = 0
        self.first_source_timestamp = None
        self.last_source_timestamp = None
        self.last_new_frame_received = None

    @property
    def ready(self):
        """Return whether both the count and PX4 source-time gates passed."""
        return (
            self.consecutive_frames >= self.min_frames and
            self.first_source_timestamp is not None and
            self.last_source_timestamp - self.first_source_timestamp >=
            self.min_source_span_us)

    def expire(self, received_at):
        """Reset when no strictly newer source frame arrived in time."""
        if (self.last_new_frame_received is not None and
                float(received_at) - self.last_new_frame_received >
                self.timeout_seconds):
            last_source_timestamp = self.last_source_timestamp
            self.reset()
            # Keep the monotonic watermark so a repeated stale frame cannot
            # become the first frame of a new window after timeout.
            self.last_source_timestamp = last_source_timestamp
            return True
        return False

    def update(self, source_timestamp, received_at, valid):
        """Consume one callback and report whether it was a new source frame."""
        received_at = float(received_at)
        self.expire(received_at)
        if not valid:
            self.reset()
            return False
        source_timestamp = int(source_timestamp)
        if source_timestamp <= 0:
            self.reset()
            return False
        if (self.last_source_timestamp is not None and
                source_timestamp <= self.last_source_timestamp):
            return False
        if self.consecutive_frames == 0:
            self.first_source_timestamp = source_timestamp
        self.last_source_timestamp = source_timestamp
        self.last_new_frame_received = received_at
        self.consecutive_frames += 1
        return True


class MissionLogic:
    """Pure mission state machine; ROS callbacks only update its inputs."""

    def __init__(self, mission_mode='hover_test', target_altitude=1.5,
                 simulation_mode=False, competition_mode=False,
                 enable_control=False, enable_auto_arm=False,
                 enable_visual_follow=True, enable_payload_release=False,
                 enable_dynamic_landing=False, enable_auto_disarm=False,
                 enable_second_takeoff=False, prestream_cycles=40,
                 hover_confirm_seconds=3.0, hover_test_seconds=10.0,
                 dwell_on_car_seconds=5.0, mission_timeout_seconds=90.0,
                 b_deadline_seconds=15.0, altitude_tolerance=0.1,
                 stable_seconds=1.0, visual_stable_seconds=0.5,
                 payload_ack_timeout=3.0, dynamic_near_height_m=0.6,
                 mission_deadline_s=None, point_b_progress=None,
                 point_d_progress=None, return_reserve_s=10.0,
                 hover_altitude_m=1.5, hover_altitude_tolerance_m=0.1,
                 hover_max_vertical_speed_mps=0.15,
                 progress_timeout_seconds=2.0,
                 point_b_deadline_s=15.0, point_b_soft_deadline_s=13.0,
                 point_d_deadline_s=52.0, payload_latest_command_s=49.0):
        self.mode = parse_mission_mode(mission_mode)
        self.mode_name = str(mission_mode)
        self.target_altitude = float(target_altitude)
        self.simulation_mode = bool(simulation_mode)
        self.competition_mode = bool(competition_mode)
        self.enable_control = bool(enable_control)
        self.enable_auto_arm = bool(enable_auto_arm)
        self.enable_visual_follow = bool(enable_visual_follow)
        self.enable_payload_release = bool(enable_payload_release)
        self.enable_dynamic_landing = bool(enable_dynamic_landing)
        self.enable_auto_disarm = bool(enable_auto_disarm)
        self.enable_second_takeoff = bool(enable_second_takeoff)
        self.prestream_cycles = int(prestream_cycles)
        self.hover_confirm_seconds = float(hover_confirm_seconds)
        self.hover_test_seconds = float(hover_test_seconds)
        self.dwell_on_car_seconds = max(5.0, float(dwell_on_car_seconds))
        self.mission_timeout_seconds = float(
            mission_timeout_seconds if mission_deadline_s is None
            else mission_deadline_s)
        self.b_deadline_seconds = float(b_deadline_seconds)
        self.altitude_tolerance = float(altitude_tolerance)
        self.stable_seconds = float(stable_seconds)
        self.visual_stable_seconds = float(visual_stable_seconds)
        self.payload_ack_timeout = float(payload_ack_timeout)
        self.dynamic_near_height_m = float(dynamic_near_height_m)
        self.point_b_progress = (None if point_b_progress is None else
                                 int(point_b_progress))
        self.point_d_progress = (None if point_d_progress is None else
                                 int(point_d_progress))
        self.return_reserve_s = float(return_reserve_s)
        self.hover_altitude_m = float(hover_altitude_m)
        self.hover_altitude_tolerance_m = float(hover_altitude_tolerance_m)
        self.hover_max_vertical_speed_mps = float(hover_max_vertical_speed_mps)
        self.progress_timeout_seconds = float(progress_timeout_seconds)
        self.point_b_deadline_s = float(point_b_deadline_s)
        self.point_b_soft_deadline_s = float(point_b_soft_deadline_s)
        self.point_d_deadline_s = float(point_d_deadline_s)
        self.payload_latest_command_s = float(payload_latest_command_s)
        if (self.return_reserve_s < 0.0 or
                self.hover_altitude_tolerance_m < 0.0 or
                self.hover_max_vertical_speed_mps <= 0.0 or
                self.progress_timeout_seconds <= 0.0 or
                self.point_b_soft_deadline_s < 0.0 or
                self.point_b_deadline_s <= self.point_b_soft_deadline_s or
                self.point_d_deadline_s <= self.point_b_deadline_s or
                self.payload_latest_command_s < self.point_b_deadline_s or
                self.payload_latest_command_s >= self.point_d_deadline_s or
                self.mission_timeout_seconds <= self.point_d_deadline_s):
            raise ValueError('invalid drop mission timing or hover limits')
        self._handlers = {
            S.WAIT_PX4.value: self._wait_px4,
            S.WAIT_SAFETY.value: self._wait_safety,
            S.WAIT_START.value: self._wait_start,
            S.PRESTREAM.value: self._prestream,
            S.REQUEST_OFFBOARD.value: self._request_offboard,
            S.ARMING.value: self._wait_armed,
            S.WAIT_MANUAL_ARM.value: self._wait_armed,
            S.TAKEOFF.value: self._takeoff,
            S.HOVER_150CM.value: self._hover_150,
            S.HOVER_3S.value: self._hover_3s,
            S.SEARCH_CAR.value: self._search_car,
            S.VISION_FOLLOW.value: self._vision_follow,
            S.ALIGN_FOR_DROP.value: self._align_for_drop,
            S.PAYLOAD_RELEASE.value: self._payload_release,
            S.WAIT_RELEASE_ACK.value: self._wait_release_ack,
            S.ALIGN_PLATFORM.value: self._align_platform,
            S.DYNAMIC_DESCENT_HIGH.value: self._descent_high,
            S.DYNAMIC_DESCENT_NEAR.value: self._descent_near,
            S.TOUCHDOWN_CHECK.value: self._touchdown_check,
            S.LANDED_ON_CAR.value: self._landed_on_car,
            S.DWELL_5S.value: self._dwell,
            S.DISARM_ON_CAR.value: self._disarm_on_car,
            S.SECOND_PRESTREAM.value: self._second_prestream,
            S.SECOND_ARM.value: self._second_arm,
            S.SECOND_TAKEOFF.value: self._second_takeoff,
            S.RETURN_HOME.value: self._return_home,
            S.FINAL_LAND.value: self._final_land,
        }
        self.reset()

    def reset(self):
        self.state = S.WAIT_PX4.value
        self.state_since = 0.0
        self.started_at = None
        self.h = None
        self.position = None
        self.velocity = (0.0, 0.0, 0.0)
        self.heading = 0.0
        self.px4_fresh = False
        self.odom_ready = False
        self.attitude_valid = False
        self.abnormal_tilt = False
        self.failsafe = False
        self.armed = False
        self.nav_state = 0
        self.offboard = False
        self.ever_offboard = False
        self.start_signal = False
        self.start_time_hint = None
        self.safety_ready = False
        self.preflight_ok = False
        self.car_progress = CarProgress.UNKNOWN_OR_IDLE
        self.car_regression = False
        self.last_car_progress_at = None
        self.target_ok = False
        self.aligned = False
        self.visual_stable_since = None
        self.aligned_stable_since = None
        self.touchdown_candidate = False
        self.touchdown = False
        self.payload_sent = False
        self.payload_ack = False
        self.payload_failure = None
        self.formed_follow_before_b = False
        self.b_deadline_failed = False
        self.b_soft_deadline_warned = False
        self.payload_forbidden = False
        self.d_deadline_failed = False
        self.d_deadline_passed = False
        self.completed_before_d = False
        self.second_cycle = False
        self.disarm_confirmed = False
        self.landed = False
        self.event = 'RESET'
        self.safety_block = 'WAIT_PX4'
        self.stable_since = None
        self.prestream_count = 0

    def transition(self, state, now, event=None):
        state = state.value if isinstance(state, MissionState) else str(state)
        if state not in (item.value for item in MissionState):
            raise ValueError('unknown mission state: ' + state)
        self.state = state
        self.state_since = float(now)
        self.stable_since = None
        if event:
            self.event = event

    @property
    def cruise_z(self):
        return None if self.h is None else self.h[2] - self.target_altitude

    @property
    def relative_h_height(self):
        if self.h is None or self.position is None:
            return 0.0
        return self.h[2] - self.position[2]

    def remaining_seconds(self, now):
        if self.started_at is None:
            return self.mission_timeout_seconds
        return max(0.0, self.mission_timeout_seconds - (now - self.started_at))

    def elapsed_seconds(self, now):
        if self.started_at is None:
            return 0.0
        return max(0.0, float(now) - self.started_at)

    def dwell_progress(self, now):
        if self.state != S.DWELL_5S.value:
            return 1.0 if self.state in (
                S.DISARM_ON_CAR.value, S.SECOND_PRESTREAM.value,
                S.SECOND_ARM.value, S.SECOND_TAKEOFF.value) else 0.0
        return min(1.0, max(0.0, (now - self.state_since) /
                            self.dwell_on_car_seconds))

    def update_position(self, x, y, z, vx, vy, vz, heading, valid):
        values = (x, y, z, vx, vy, vz, heading)
        self.px4_fresh = bool(valid) and all(math.isfinite(v) for v in values)
        if self.px4_fresh:
            self.position = tuple(float(v) for v in (x, y, z))
            self.velocity = tuple(float(v) for v in (vx, vy, vz))
            self.heading = float(heading)

    def update_visual(self, valid, aligned, now):
        self.target_ok = bool(valid)
        self.aligned = self.target_ok and bool(aligned)
        if not self.target_ok:
            self.visual_stable_since = None
            self.aligned_stable_since = None
            self.safety_block = 'VISION_INVALID'
        elif self.visual_stable_since is None:
            self.visual_stable_since = float(now)
        if not self.aligned:
            self.aligned_stable_since = None
        elif self.aligned_stable_since is None:
            self.aligned_stable_since = float(now)

    def visual_stable(self, now):
        return (self.target_ok and self.visual_stable_since is not None and
                now - self.visual_stable_since + 1e-9 >=
                self.visual_stable_seconds)

    def alignment_stable(self, now):
        return (self.visual_stable(now) and self.aligned and
                self.aligned_stable_since is not None and
                now - self.aligned_stable_since + 1e-9 >=
                self.visual_stable_seconds)

    def lock_home(self):
        if self.h is not None:
            return True
        if (not self.px4_fresh or not self.odom_ready or
                self.position is None):
            return False
        if math.hypot(self.velocity[0], self.velocity[1]) >= 0.15:
            return False
        if abs(self.velocity[2]) >= 0.15:
            return False
        self.h = (*self.position, self.heading)
        return True

    def record_prestream_cycle(self):
        if self.state in (S.PRESTREAM.value, S.SECOND_PRESTREAM.value):
            self.prestream_count += 1

    def update_status(self, armed, nav_state, offboard, failsafe, now=0.0,
                      preflight_ok=True):
        self.armed = bool(armed)
        self.nav_state = int(nav_state)
        self.offboard = bool(offboard)
        self.failsafe = bool(failsafe)
        self.preflight_ok = bool(preflight_ok)
        if self.offboard:
            self.ever_offboard = True
        elif self.ever_offboard and self.state not in (
                S.LANDED_ON_CAR.value, S.DWELL_5S.value,
                S.DISARM_ON_CAR.value, S.SECOND_PRESTREAM.value,
                S.FINAL_LAND.value, S.COMPLETE.value,
                S.DATA_TIMEOUT.value, S.FAILSAFE.value) and not (
                    self.second_cycle and
                    self.state == S.REQUEST_OFFBOARD.value and
                    self.disarm_confirmed):
            self.safety_block = 'OFFBOARD_EXIT'
            self.transition(S.FAILSAFE, now, 'PX4_EXITED_OFFBOARD')

    def update_landed(self, landed):
        """Track PX4 landed state for the final completion gate."""
        self.landed = bool(landed)

    def update_car_progress(self, value):
        return self._update_car_progress(value, None)

    def update_car_progress_at(self, value, now):
        return self._update_car_progress(value, float(now))

    def _update_car_progress(self, value, now):
        try:
            value = CarProgress(int(value))
        except (TypeError, ValueError):
            self.event = 'CAR_PROGRESS_INVALID'
            return False
        if value < self.car_progress:
            self.car_regression = True
            self.safety_block = 'CAR_PROGRESS_REGRESSION'
            self.event = 'CAR_PROGRESS_REGRESSION'
            return False
        if value == self.car_progress:
            if now is not None:
                self.last_car_progress_at = now
            return False
        self.car_progress = value
        if now is not None:
            self.last_car_progress_at = now
        return True

    def auto_arm_allowed(self):
        base = all((self.enable_control, self.enable_auto_arm,
                    self.start_signal, self.px4_fresh, self.h is not None,
                    not self.failsafe, not self.abnormal_tilt,
                    self.preflight_ok))
        if self.simulation_mode:
            return base
        return (base and self.competition_mode and self.safety_ready and
                self.attitude_valid)

    def auto_disarm_allowed(self, physical_touchdown=False):
        if not self.enable_auto_disarm:
            self.safety_block = 'AUTO_DISARM_DISABLED'
            return False
        if not self.touchdown:
            return False
        if self.simulation_mode:
            return True
        if not physical_touchdown:
            self.safety_block = 'PHYSICAL_TOUCHDOWN_REQUIRED'
            return False
        return True

    def altitude_reached(self):
        return (self.position is not None and self.cruise_z is not None and
                abs(self.position[2] - self.cruise_z) <=
                self.altitude_tolerance and
                math.sqrt(sum(v * v for v in self.velocity)) < 0.25)

    def hover_stable_condition(self):
        return (self.position is not None and self.cruise_z is not None and
                abs(self.position[2] - self.cruise_z) <=
                self.hover_altitude_tolerance_m and
                abs(self.velocity[2]) <= self.hover_max_vertical_speed_mps)

    def stable(self, condition, now, seconds):
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = float(now)
        return now - self.stable_since >= seconds

    def step(self, now):
        now = float(now)
        if self.state in (S.DATA_TIMEOUT.value, S.FAILSAFE.value,
                          S.COMPLETE.value):
            return
        if self.failsafe or self.abnormal_tilt or self.car_regression:
            reason = 'PX4_FAILSAFE' if self.failsafe else 'ABNORMAL_TILT'
            if self.car_regression:
                reason = 'CAR_PROGRESS_REGRESSION'
            self.safety_block = reason
            self.transition(S.FAILSAFE, now, reason)
            return
        if (self.mode_name == 'drop' and self.state in (
                S.ALIGN_PLATFORM.value, S.DYNAMIC_DESCENT_HIGH.value,
                S.DYNAMIC_DESCENT_NEAR.value, S.TOUCHDOWN_CHECK.value,
                S.LANDED_ON_CAR.value, S.DWELL_5S.value,
                S.DISARM_ON_CAR.value, S.SECOND_PRESTREAM.value,
                S.SECOND_ARM.value, S.SECOND_TAKEOFF.value)):
            self.transition(S.RETURN_HOME, now, 'DROP_MODE_OLD_LAND_STATE')
            return
        elapsed = self.elapsed_seconds(now)
        if (self.started_at is not None and
                self.remaining_seconds(now) <= self.return_reserve_s and
                self.state not in (S.RETURN_HOME.value, S.FINAL_LAND.value)):
            self.safety_block = 'MISSION_DEADLINE_RETURN'
            self.transition(S.RETURN_HOME, now, 'MISSION_DEADLINE_RETURN')
            return
        if self.mode_name == 'drop' and self.started_at is not None:
            if (elapsed >= self.point_b_soft_deadline_s and
                    not self.formed_follow_before_b and
                    not self.b_soft_deadline_warned):
                self.b_soft_deadline_warned = True
                self.safety_block = 'B_SOFT_DEADLINE_SEARCH'
                self.event = 'B_SOFT_DEADLINE_SEARCH'
            if (elapsed >= self.point_b_deadline_s and
                    not self.formed_follow_before_b and
                    not self.b_deadline_failed):
                self.b_deadline_failed = True
                self.safety_block = 'B_POINT_DEADLINE_MISSED'
                self.event = ('B_POINT_DEADLINE_MISSED' if
                              self.point_b_progress is not None else
                              'B_FOLLOW_MILESTONE_MISSED')
            if elapsed >= self.payload_latest_command_s and not self.payload_sent:
                self.payload_forbidden = True
                self.safety_block = 'PAYLOAD_LATEST_COMMAND_MISSED'
                self.transition(S.RETURN_HOME, now,
                                'PAYLOAD_LATEST_COMMAND_MISSED')
                return
            if elapsed >= self.point_d_deadline_s:
                self.d_deadline_passed = bool(self.payload_ack)
                if not self.payload_ack:
                    self.d_deadline_failed = True
                    self.safety_block = 'D_POINT_DEADLINE_MISSED'
                    self.event = 'D_POINT_DEADLINE_MISSED'
                else:
                    self.event = 'D_POINT_DEADLINE_PASSED'
                if self.state not in (S.RETURN_HOME.value,
                                      S.FINAL_LAND.value):
                    self.transition(S.RETURN_HOME, now, self.event)
                return
        if (self.started_at is not None and
                now - self.started_at >= self.b_deadline_seconds and
                not self.formed_follow_before_b and
                not self.b_deadline_failed and
                self.state != S.RETURN_HOME.value and
                self.car_progress < CarProgress.PASSED_B):
            self.event = 'B_FOLLOW_MILESTONE_MISSED'
        if (self.mode_name == 'drop' and self.point_b_progress is not None and
                self.started_at is not None and
                self.car_progress >= self.point_b_progress and
                not self.formed_follow_before_b and
                self.state not in (S.RETURN_HOME.value, S.FINAL_LAND.value)):
            self.safety_block = 'B_FOLLOW_CUTOFF'
            self.transition(S.RETURN_HOME, now, 'B_FOLLOW_CUTOFF')
            return
        if (self.mode_name == 'drop' and self.point_b_progress is not None and
                self.last_car_progress_at is not None and
                self.started_at is not None and
                now - self.last_car_progress_at > self.progress_timeout_seconds and
                self.state not in (S.RETURN_HOME.value, S.FINAL_LAND.value)):
            self.safety_block = 'CAR_PROGRESS_TIMEOUT'
            self.transition(S.RETURN_HOME, now, 'CAR_PROGRESS_TIMEOUT')
            return
        d_progress = (CarProgress.PASSED_D if self.point_d_progress is None
                      else self.point_d_progress)
        if (self.car_progress >= d_progress and
                not self.completed_before_d and self.state in (
                    S.SEARCH_CAR.value, S.VISION_FOLLOW.value,
                    S.ALIGN_FOR_DROP.value, S.PAYLOAD_RELEASE.value,
                    S.ALIGN_PLATFORM.value, S.DYNAMIC_DESCENT_HIGH.value,
                    S.DYNAMIC_DESCENT_NEAR.value,
                    S.TOUCHDOWN_CHECK.value)):
            self.safety_block = 'D_PASSED_ABORT'
            self.transition(S.RETURN_HOME, now, 'D_PASSED_ABORT')
        handler = self._handlers.get(self.state)
        if handler is not None:
            handler(now)

    def _wait_px4(self, now):
        self.safety_block = 'WAIT_PX4'
        if self.px4_fresh and self.attitude_valid:
            self.transition(S.WAIT_SAFETY, now)

    def _wait_safety(self, now):
        self.safety_block = 'WAIT_SAFETY'
        if self.simulation_mode or (self.safety_ready and self.attitude_valid):
            self.transition(S.WAIT_START, now)

    def _wait_start(self, now):
        self.safety_block = 'WAIT_START'
        start_requested = (self.start_signal or
                           (self.mode_name == 'hover_test' and
                            self.enable_control))
        if start_requested and self.lock_home():
            self.start_signal = True
            self.started_at = (now if self.start_time_hint is None else
                               min(now, self.start_time_hint))
            self.last_car_progress_at = now
            self.prestream_count = 0
            self.safety_block = 'NONE'
            self.transition(S.PRESTREAM, now, 'MISSION_STARTED')

    def _prestream(self, now):
        if self.prestream_count >= self.prestream_cycles:
            self.transition(S.REQUEST_OFFBOARD, now)

    def _request_offboard(self, now):
        if not self.offboard:
            return
        if self.second_cycle:
            if self.armed:
                self.transition(S.SECOND_TAKEOFF, now)
            else:
                self.transition(S.SECOND_ARM, now)
            return
        if self.armed:
            self.transition(S.TAKEOFF, now)
        elif self.auto_arm_allowed() and (
                self.mode_name != 'hover_test' or self.simulation_mode):
            self.transition(S.ARMING, now)
        else:
            self.transition(S.WAIT_MANUAL_ARM, now)

    def _wait_armed(self, now):
        if self.armed and self.offboard:
            target = S.SECOND_TAKEOFF if self.second_cycle else S.TAKEOFF
            self.transition(target, now)

    def _takeoff(self, now):
        if self.stable(self.altitude_reached(), now, self.stable_seconds):
            self.transition(S.HOVER_150CM, now, 'TAKEOFF_STABLE')

    def _hover_150(self, now):
        if self.mode_name == 'hover_test':
            if now - self.state_since >= self.hover_test_seconds:
                self.transition(S.FINAL_LAND, now)
        else:
            self.transition(S.HOVER_3S, now)
            if self.hover_stable_condition():
                self.stable_since = now

    def _hover_3s(self, now):
        if not self.hover_stable_condition():
            self.stable_since = None
            return
        if self.stable_since is None:
            self.stable_since = now
        if now - self.stable_since + 1e-9 >= max(
                3.0, self.hover_confirm_seconds):
            self.transition(S.SEARCH_CAR, now)

    def _search_car(self, now):
        if self.visual_stable(now):
            self.transition(S.VISION_FOLLOW, now)
            if (self.car_progress < CarProgress.PASSED_B and
                    not self.b_deadline_failed):
                self.formed_follow_before_b = True

    def _vision_follow(self, now):
        if self.alignment_stable(now):
            target = (S.ALIGN_FOR_DROP if self.mode_name == 'drop'
                      else S.ALIGN_PLATFORM)
            self.transition(target, now)

    def _align_for_drop(self, now):
        can_release = (self.alignment_stable(now) and
                       self.altitude_reached() and
                       not self.payload_forbidden and
                       self.car_progress < (CarProgress.PASSED_D if
                                            self.point_d_progress is None else
                                            self.point_d_progress))
        if can_release and not self.enable_payload_release:
            self.safety_block = 'PAYLOAD_RELEASE_DISABLED'
            self.transition(S.FAILSAFE, now, 'PAYLOAD_RELEASE_DISABLED')
            return
        if can_release and self.enable_payload_release and not self.payload_sent:
            self.payload_sent = True
            self.completed_before_d = True
            self.transition(S.PAYLOAD_RELEASE, now, 'PAYLOAD_RELEASE')

    def _payload_release(self, now):
        self.transition(S.WAIT_RELEASE_ACK, now)

    def _wait_release_ack(self, now):
        if self.payload_ack:
            if self.elapsed_seconds(now) < self.point_d_deadline_s:
                self.d_deadline_passed = True
                self.transition(S.RETURN_HOME, now,
                                'D_POINT_DEADLINE_PASSED')
            else:
                self.transition(S.RETURN_HOME, now, 'PAYLOAD_ACK')
        elif now - self.state_since >= self.payload_ack_timeout:
            self.safety_block = 'PAYLOAD_ACK_TIMEOUT'
            self.transition(S.FAILSAFE, now, 'PAYLOAD_ACK_TIMEOUT')

    def _align_platform(self, now):
        if (self.enable_dynamic_landing and self.alignment_stable(now) and
                self.car_progress < CarProgress.PASSED_D):
            self.transition(S.DYNAMIC_DESCENT_HIGH, now)

    def _descent_high(self, now):
        if self.touchdown_candidate:
            self.transition(S.TOUCHDOWN_CHECK, now)
        elif (self.visual_stable(now) and
              self.relative_h_height <= self.dynamic_near_height_m):
            self.transition(S.DYNAMIC_DESCENT_NEAR, now)

    def _descent_near(self, now):
        if self.touchdown_candidate:
            self.transition(S.TOUCHDOWN_CHECK, now)

    def _touchdown_check(self, now):
        if self.touchdown:
            self.completed_before_d = (
                self.car_progress < CarProgress.PASSED_D)
            self.transition(S.LANDED_ON_CAR, now, 'TOUCHDOWN_CONFIRMED')
        elif not self.touchdown_candidate:
            self.transition(S.DYNAMIC_DESCENT_NEAR, now)

    def _landed_on_car(self, now):
        self.transition(S.DWELL_5S, now)

    def _dwell(self, now):
        if now - self.state_since >= self.dwell_on_car_seconds:
            self.transition(S.DISARM_ON_CAR, now)

    def _disarm_on_car(self, now):
        if not self.armed:
            self.disarm_confirmed = True
            if self.enable_second_takeoff:
                self.second_cycle = True
                self.prestream_count = 0
                self.transition(S.SECOND_PRESTREAM, now)
            else:
                self.transition(S.COMPLETE, now)

    def _second_prestream(self, now):
        if not (self.disarm_confirmed and self.px4_fresh and
                self.visual_stable(now)):
            return
        if self.prestream_count >= self.prestream_cycles:
            # Disarming normally exits Offboard. Re-request it after the
            # complete second prestream before issuing the second ARM.
            self.transition(S.REQUEST_OFFBOARD, now)

    def _second_arm(self, now):
        if self.armed and self.offboard:
            self.transition(S.SECOND_TAKEOFF, now)

    def _second_takeoff(self, now):
        if self.stable(self.altitude_reached(), now, self.stable_seconds):
            self.transition(S.RETURN_HOME, now)

    def _return_home(self, now):
        if self.at_h():
            self.transition(S.FINAL_LAND, now)

    def _final_land(self, now):
        if self.landed and not self.armed:
            self.transition(S.COMPLETE, now, 'MISSION_COMPLETE')

    def at_h(self):
        if self.h is None or self.position is None:
            return False
        distance = math.hypot(self.position[0] - self.h[0],
                              self.position[1] - self.h[1])
        speed = math.sqrt(sum(v * v for v in self.velocity))
        return (distance <= 0.25 and
                abs(self.position[2] - self.cruise_z) <=
                self.altitude_tolerance and speed <= 0.25)

    def reset_if_safe(self):
        if not self.armed and self.state in (
                S.WAIT_PX4.value, S.WAIT_START.value):
            self.reset()
            return True
        return False

    def request_safe_abort(self, now):
        """Route an active armed mission through return and normal landing."""
        if not self.armed or self.state in (
                S.COMPLETE.value, S.DATA_TIMEOUT.value, S.FAILSAFE.value,
                S.FINAL_LAND.value):
            return False
        target = (
            S.RETURN_HOME if self.h is not None and self.position is not None
            else S.FINAL_LAND)
        self.safety_block = 'MISSION_ABORT_REQUESTED'
        self.transition(target, now, 'MISSION_ABORT_REQUESTED')
        return True


def quaternion_is_valid(values, norm_tolerance=0.1):
    """Return whether values form a finite, approximately unit quaternion."""
    try:
        quaternion = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return False
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        return False
    norm = math.sqrt(sum(value * value for value in quaternion))
    return abs(norm - 1.0) <= float(norm_tolerance)


def prestream_is_safe(enable_control, prestream_cycles):
    """Flight may only be enabled with at least 20 actual cycles."""
    return not bool(enable_control) or int(prestream_cycles) >= 20


def control_output_allowed(state, px4_stale):
    """No PX4 output is allowed after terminal safety faults."""
    return not px4_stale and state not in (
        S.DATA_TIMEOUT.value, S.FAILSAFE.value, S.COMPLETE.value)


def data_loss_action(armed, status_stale):
    """Choose the bounded response to critical PX4 input loss."""
    if bool(armed) and not bool(status_stale):
        return S.FINAL_LAND.value
    return S.DATA_TIMEOUT.value


def vehicle_command_allowed(enable_control, state, px4_stale=False):
    """Return whether the adapter may consider a PX4 command."""
    command_states = (
        S.REQUEST_OFFBOARD.value, S.ARMING.value, S.SECOND_ARM.value,
        S.DISARM_ON_CAR.value, S.FINAL_LAND.value)
    return (bool(enable_control) and
            control_output_allowed(state, px4_stale) and
            state in command_states)


def descent_speed_for_state(state, high_speed, near_speed, contact_speed):
    """Select the bounded descent phase speed; other states do not descend."""
    speeds = {
        S.DYNAMIC_DESCENT_HIGH.value: float(high_speed),
        S.DYNAMIC_DESCENT_NEAR.value: float(near_speed),
        S.TOUCHDOWN_CHECK.value: float(contact_speed),
    }
    return speeds.get(state, 0.0)


def vision_is_fresh(now, received_at, target_age_ms, timeout_s,
                    max_target_age_ms=250.0):
    values = (now, target_age_ms, timeout_s, max_target_age_ms)
    if received_at is None or not all(math.isfinite(float(v)) for v in values):
        return False
    if not math.isfinite(float(received_at)):
        return False
    return (0.0 <= now - received_at <= timeout_s and
            0.0 <= target_age_ms <= max_target_age_ms)


def payload_ack_is_new(release_time, ack_time, ack_value):
    """Accept only a true acknowledgement received after release."""
    return bool(ack_value) and release_time is not None and ack_time > release_time
