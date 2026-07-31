"""Finite, read-only PX4 preflight readiness gate for SITL runners."""

import json
import time

from px4_msgs.msg import VehicleStatus
import rclpy
from rclpy.node import Node

from uav_control.hover_matrix import PreflightStability

from .px4_qos import px4_output_qos


class SitlPreflightGate(Node):
    """Observe VehicleStatus without publishing anything."""

    def __init__(self):
        super().__init__('sitl_preflight_gate')
        self.declare_parameter('timeout_seconds', 15.0)
        self.declare_parameter('stable_seconds', 3.0)
        self.declare_parameter('max_message_age_seconds', 1.0)
        self.started_at = time.monotonic()
        self.checker = PreflightStability(
            self.get_parameter('stable_seconds').value,
            self.get_parameter('max_message_age_seconds').value)
        self.passed = False
        qos = px4_output_qos(depth=10)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._status, qos)

    def _status(self, msg):
        if self.checker.update(
                msg.timestamp, msg.pre_flight_checks_pass,
                time.monotonic()):
            self.passed = True

    def timed_out(self):
        now = time.monotonic()
        self.checker.expired(now)
        return now - self.started_at >= float(
            self.get_parameter('timeout_seconds').value)


def main(args=None):
    """Wait for readiness and return 5 on a finite refusal."""
    rclpy.init(args=args)
    node = SitlPreflightGate()
    try:
        while rclpy.ok() and not node.passed and not node.timed_out():
            rclpy.spin_once(node, timeout_sec=0.1)
        summary = {
            'passed': node.passed,
            'reason': (
                'pre_flight_checks_pass continuously true'
                if node.passed else node.checker.reason),
            'last_vehicle_status': node.checker.last,
        }
        print(json.dumps(summary, sort_keys=True), flush=True)
        return 0 if node.passed else 5
    finally:
        node.destroy_node()
        rclpy.shutdown()
