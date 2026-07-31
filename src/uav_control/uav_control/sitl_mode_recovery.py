"""Restore a landed, disarmed PX4 SITL from Offboard to Position mode."""

import json
import time

from px4_msgs.msg import (
    VehicleCommand, VehicleCommandAck, VehicleLandDetected, VehicleStatus)
import rclpy
from rclpy.node import Node

from uav_control.hover_matrix import (
    mode_recovery_safe, mode_recovery_status_valid, PreflightStability)

from .px4_qos import px4_input_qos, px4_output_qos


OFFBOARD_NAV_STATE = 14
POSITION_MAIN_MODE = 3.0


class SitlModeRecovery(Node):
    """Issue no command until independent landed and Disarmed gates pass."""

    def __init__(self):
        super().__init__('sitl_mode_recovery')
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('timeout_seconds', 15.0)
        self.declare_parameter('safe_stable_seconds', 1.0)
        self.declare_parameter('preflight_stable_seconds', 3.0)
        self.declare_parameter('max_message_age_seconds', 1.0)
        self.started = time.monotonic()
        self.status = None
        self.land = None
        self.status_received = None
        self.land_received = None
        self.status_timestamp = None
        self.land_timestamp = None
        self.safe_since = None
        self.command_sent_at = None
        self.attempts = 0
        self.ack = None
        self.exit_code = None
        self.reason = 'waiting for landed and Disarmed'
        self.verifier = PreflightStability(
            self.get_parameter('preflight_stable_seconds').value,
            self.get_parameter('max_message_age_seconds').value)
        qos = px4_output_qos(depth=10)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_input_qos(depth=10))
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._status, qos)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._land, qos)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self._ack, qos)
        self.create_timer(0.1, self._tick)

    def _status(self, msg):
        now = time.monotonic()
        if (self.status_timestamp is not None and
                msg.timestamp <= self.status_timestamp):
            return
        self.status_timestamp = msg.timestamp
        self.status_received = now
        self.status = msg
        if self.command_sent_at is not None:
            valid = mode_recovery_status_valid(
                self.ack,
                VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
                msg.arming_state, VehicleStatus.ARMING_STATE_DISARMED,
                msg.nav_state, msg.nav_state_user_intention, msg.failsafe,
                self.land is not None and self.land.landed,
                self._fresh(self.land_received, now),
                msg.pre_flight_checks_pass)
            if self.verifier.update(msg.timestamp, valid, now):
                self._finish(0, 'Position mode restored and status stable')

    def _land(self, msg):
        if (self.land_timestamp is not None and
                msg.timestamp <= self.land_timestamp):
            return
        self.land_timestamp = msg.timestamp
        self.land_received = time.monotonic()
        self.land = msg

    def _ack(self, msg):
        if msg.command != VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            return
        self.ack = int(msg.result)
        if self.ack not in (
                VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
                VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS):
            self._finish(9, 'PX4 rejected mode recovery, ACK=%d' % self.ack)

    def _fresh(self, received, now):
        return (
            received is not None and
            now - received <=
            self.get_parameter('max_message_age_seconds').value)

    def _publish_mode(self, now):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = 1.0
        msg.param2 = POSITION_MAIN_MODE
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        self.command_pub.publish(msg)
        self.command_sent_at = now
        self.attempts += 1

    def _tick(self):
        if self.exit_code is not None:
            return
        if not self.get_parameter('simulation_mode').value:
            self._finish(
                9, 'simulation_mode=true is required for mode recovery')
            return
        now = time.monotonic()
        if now - self.started >= self.get_parameter('timeout_seconds').value:
            self._finish(9, self.reason + '; recovery timeout')
            return
        status_fresh = self._fresh(self.status_received, now)
        land_fresh = self._fresh(self.land_received, now)
        armed = (
            self.status is None or
            self.status.arming_state !=
            VehicleStatus.ARMING_STATE_DISARMED)
        landed = self.land is not None and self.land.landed
        if self.status is not None and self.status.failsafe:
            self._finish(9, 'failsafe active; mode recovery prohibited')
            return
        if self.command_sent_at is not None and (armed or not landed):
            self._finish(
                9, 'airborne or armed state observed after recovery started')
            return
        if self.command_sent_at is None:
            if not mode_recovery_safe(
                    armed, landed, status_fresh, land_fresh):
                self.safe_since = None
                self.reason = (
                    'waiting for fresh landed=true and arming_state=Disarmed')
                return
            if self.safe_since is None:
                self.safe_since = now
                return
            if now - self.safe_since < self.get_parameter(
                    'safe_stable_seconds').value:
                return
            self._publish_mode(now)
            self.reason = 'waiting for mode recovery ACK and stable status'
            return
        if (self.ack != VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED and
                now - self.command_sent_at >= 1.0):
            if self.attempts >= 3:
                self._finish(9, 'mode recovery ACK timeout')
            else:
                self._publish_mode(now)
        self.verifier.expired(now)

    def _finish(self, code, reason):
        if self.exit_code is None:
            self.exit_code = int(code)
            self.reason = reason
            print(json.dumps({
                'passed': code == 0,
                'reason': reason,
                'attempts': self.attempts,
                'mode_recovery_ack': self.ack,
                'final_arming_state': (
                    None if self.status is None else
                    int(self.status.arming_state)),
                'final_nav_state': (
                    None if self.status is None else
                    int(self.status.nav_state)),
                'final_nav_state_user_intention': (
                    None if self.status is None else
                    int(self.status.nav_state_user_intention)),
                'final_failsafe': (
                    None if self.status is None else
                    bool(self.status.failsafe)),
                'final_landed': (
                    None if self.land is None else bool(self.land.landed)),
                'final_pre_flight_checks_pass': (
                    None if self.status is None else
                    bool(self.status.pre_flight_checks_pass)),
            }, sort_keys=True), flush=True)
            rclpy.shutdown()


def main(args=None):
    """Run the bounded post-flight recovery."""
    rclpy.init(args=args)
    node = SitlModeRecovery()
    try:
        rclpy.spin(node)
    finally:
        code = node.exit_code if node.exit_code is not None else 9
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
