"""Validate one hover flight and write a machine-readable result."""

import json
from pathlib import Path
import time

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

from uav_control.mission_schema import TELEMETRY

from .hover_matrix import HoverMetrics
from .px4_qos import px4_output_qos


class SitlResultRecorder(Node):
    """Observe, validate and record a SITL-only hover scenario."""

    def __init__(self):
        super().__init__('sitl_result_recorder')
        defaults = {
            'scenario': 'nominal',
            'result_file': '',
            'timeout_seconds': 90.0,
            'target_height_m': 1.0,
            'commanded_hover_duration_s': 10.0,
            'altitude_tolerance_m': 0.1,
            'abort_grace_seconds': 30.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.started_monotonic = time.monotonic()
        self.initial_status_seen = False
        self.initial_disarmed = False
        self.initial_preflight_ok = False
        self.local_position_valid_seen = False
        self.state = 'UNKNOWN'
        self.last_status = None
        self.saw_armed = False
        self.saw_offboard = False
        self.reached_target = False
        self.failsafe_seen = False
        self.metrics = HoverMetrics(
            self.get_parameter('target_height_m').value,
            self.get_parameter('altitude_tolerance_m').value)
        self.pending_failure_reason = None
        self.abort_requested_at = None
        self.exit_code = None
        qos = px4_output_qos()
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._status, qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._position, qos)
        self.create_subscription(
            String, '/uav/mission/state', self._state, 10)
        self.create_subscription(
            Float32MultiArray, '/uav/mission/telemetry',
            self._telemetry, 10)
        self.abort_pub = self.create_publisher(
            Bool, '/uav/mission/abort', 10)
        self.create_timer(0.1, self._tick)

    def elapsed(self):
        return time.monotonic() - self.started_monotonic

    def _status(self, msg):
        self.last_status = msg
        self.saw_armed = self.saw_armed or (
            msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.saw_offboard = self.saw_offboard or (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.failsafe_seen = self.failsafe_seen or bool(msg.failsafe)
        if not self.initial_status_seen:
            self.initial_status_seen = True
            self.initial_disarmed = (
                msg.arming_state == VehicleStatus.ARMING_STATE_DISARMED)
            self.initial_preflight_ok = bool(msg.pre_flight_checks_pass)
            if not self.initial_disarmed:
                self._fail('scenario did not start Disarmed')
            elif not self.initial_preflight_ok:
                self._fail('pre_flight_checks_pass was false at start')

    def _position(self, msg):
        valid = all((
            msg.xy_valid, msg.z_valid, msg.v_xy_valid, msg.v_z_valid))
        self.local_position_valid_seen = (
            self.local_position_valid_seen or valid)
        if not valid:
            self.metrics.update_height(self.state, None, self.elapsed())

    def _telemetry(self, msg):
        height = None
        if len(msg.data) > TELEMETRY['relative_h_height']:
            height = msg.data[TELEMETRY['relative_h_height']]
        self.metrics.update_height(self.state, height, self.elapsed())

    def _state(self, msg):
        now = self.elapsed()
        previous = self.state
        self.state = msg.data
        self.metrics.update_state(previous, self.state, now)
        if self.state == 'HOVER_150CM' and previous != self.state:
            self.reached_target = True
        if self.state == 'COMPLETE':
            if self.pending_failure_reason:
                self._finish(False, self.pending_failure_reason)
            else:
                self._finish_from_final_status()
        elif self.state in ('FAILSAFE', 'DATA_TIMEOUT'):
            self._fail('controller entered ' + self.state)

    def _tick(self):
        now = self.elapsed()
        if (self.pending_failure_reason is None and
                now > self.get_parameter('timeout_seconds').value):
            self._fail('timeout in state ' + self.state)
        if (self.pending_failure_reason and
                self.abort_requested_at is not None and
                now - self.abort_requested_at >
                self.get_parameter('abort_grace_seconds').value):
            self._finish(
                False, self.pending_failure_reason +
                '; safe abort did not complete before grace timeout')

    def _fail(self, reason):
        if self.exit_code is not None or self.pending_failure_reason:
            return
        self.pending_failure_reason = reason
        armed = (
            self.last_status is not None and
            self.last_status.arming_state ==
            VehicleStatus.ARMING_STATE_ARMED)
        if armed and self.state not in ('COMPLETE',):
            abort = Bool()
            abort.data = True
            self.abort_pub.publish(abort)
            self.abort_requested_at = self.elapsed()
            self.get_logger().error(
                reason + '; requested controller safe abort/landing')
        else:
            self._finish(False, reason)

    def _finish_from_final_status(self):
        status = self.last_status
        final_disarmed = (
            status is not None and status.arming_state ==
            VehicleStatus.ARMING_STATE_DISARMED)
        checks = {
            'initial Disarmed': self.initial_disarmed,
            'initial preflight checks': self.initial_preflight_ok,
            'valid local position': self.local_position_valid_seen,
            'automatic arm observed': self.saw_armed,
            'Offboard observed': self.saw_offboard,
            'target height reached': self.reached_target,
            'no failsafe': not self.failsafe_seen,
            'automatic disarm': self.saw_armed and final_disarmed,
        }
        failures = [name for name, passed in checks.items() if not passed]
        self._finish(
            not failures,
            'complete' if not failures else
            'failed checks: ' + ', '.join(failures))

    def _finish(self, passed, reason):
        if self.exit_code is not None:
            return
        now = self.elapsed()
        status = self.last_status
        target = self.get_parameter('target_height_m').value
        commanded = self.get_parameter(
            'commanded_hover_duration_s').value
        metrics = self.metrics.finish(now, commanded)
        if passed and not metrics['hover_stability_passed']:
            passed = False
            reason = 'stable hover duration was below commanded duration'
        result = {
            'scenario': self.get_parameter('scenario').value,
            'target_height_m': float(target),
            'commanded_hover_duration_s': float(
                self.get_parameter(
                    'commanded_hover_duration_s').value),
            # Compatibility: from HOVER_150CM entry until COMPLETE, including
            # FINAL_LAND and disarm. New consumers must use the named metrics.
            **metrics,
            'initial_disarmed': self.initial_disarmed,
            'auto_disarmed': (
                self.saw_armed and status is not None and
                status.arming_state ==
                VehicleStatus.ARMING_STATE_DISARMED),
            'failsafe': self.failsafe_seen,
            'final_state': self.state,
            'passed': bool(passed),
            'reason': reason,
        }
        destination = self.get_parameter('result_file').value
        if destination:
            path = Path(destination).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2) + '\n',
                            encoding='utf-8')
        self.get_logger().info(json.dumps(result, sort_keys=True))
        self.exit_code = 0 if passed else 1
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SitlResultRecorder()
    try:
        rclpy.spin(node)
    finally:
        code = node.exit_code if node.exit_code is not None else 1
        node.destroy_node()
    raise SystemExit(code)
