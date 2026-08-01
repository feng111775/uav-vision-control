"""Idempotent GPIO18 payload actuator with a software-only dry-run path."""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.declare_parameter('command_topic', '/servo_command')
        self.declare_parameter('result_topic', '/servo/result')
        self.declare_parameter('gpio_pin', 18)
        self.declare_parameter('servo_dry_run', True)
        self.declare_parameter('safe_angle_deg', 10.0)
        self.declare_parameter('release_angle_deg', 70.0)
        self.declare_parameter('prepare_duration_sec', 0.5)
        self.declare_parameter('release_duration_sec', 0.8)
        self.declare_parameter('return_duration_sec', 0.5)
        self.declare_parameter('min_pulse_us', 1000)
        self.declare_parameter('max_pulse_us', 2000)
        self.pin = int(self.get_parameter('gpio_pin').value)
        self.dry_run = bool(self.get_parameter('servo_dry_run').value)
        if self.pin != 18:
            raise ValueError('payload servo must use BCM GPIO18')
        self.result_pub = self.create_publisher(
            String, self.get_parameter('result_topic').value, 10)
        self.create_subscription(
            String, self.get_parameter('command_topic').value,
            self._command, 10)
        self._seen = set()
        self._pi = None
        if not self.dry_run:
            import pigpio
            self._pi = pigpio.pi()
            if self._pi is None or not self._pi.connected:
                raise RuntimeError('pigpiod connection failed')
        self.get_logger().warning(
            'servo dry-run enabled' if self.dry_run else
            'servo GPIO18 real output enabled')

    def _pulse(self, angle):
        low = int(self.get_parameter('min_pulse_us').value)
        high = int(self.get_parameter('max_pulse_us').value)
        pulse = round(low + (high - low) * float(angle) / 180.0)
        if self.dry_run:
            return
        if self._pi.set_servo_pulsewidth(self.pin, pulse) != 0:
            raise RuntimeError('pigpio set_servo_pulsewidth failed')

    def _publish(self, sequence_id, status, detail=''):
        message = String()
        message.data = json.dumps({
            'sequence_id': int(sequence_id),
            'status': status,
            'detail': detail,
            'stamp_ns': self.get_clock().now().nanoseconds,
        }, separators=(',', ':'))
        self.result_pub.publish(message)

    def _command(self, message):
        try:
            request = json.loads(message.data)
            sequence_id = int(request['sequence_id'])
            command = str(request['command']).lower()
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            return
        if sequence_id <= 0 or command != 'throw':
            self._publish(sequence_id, 'REJECTED', 'invalid_request')
            return
        if sequence_id in self._seen:
            self._publish(sequence_id, 'DUPLICATE', 'sequence_already_executed')
            return
        self._seen.add(sequence_id)
        try:
            self._pulse(float(self.get_parameter('safe_angle_deg').value))
            time.sleep(float(self.get_parameter('prepare_duration_sec').value))
            self._pulse(float(self.get_parameter('release_angle_deg').value))
            time.sleep(float(self.get_parameter('release_duration_sec').value))
            self._pulse(float(self.get_parameter('safe_angle_deg').value))
            time.sleep(float(self.get_parameter('return_duration_sec').value))
            if not self.dry_run:
                self._pi.set_servo_pulsewidth(self.pin, 0)
            self._publish(sequence_id, 'SUCCESS')
        except Exception as exc:
            self._publish(sequence_id, 'FAILED', str(exc))

    def destroy_node(self):
        if self._pi is not None:
            try:
                self._pi.set_servo_pulsewidth(self.pin, 0)
            finally:
                self._pi.stop()
                self._pi = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
