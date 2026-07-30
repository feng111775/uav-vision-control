"""Normalize simulated car starts into idempotent mission events."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .stage4c_core import StartGate


class CarStartGateway(Node):
    """Simulation gateway; it has no flight-control publisher."""

    def __init__(self):
        super().__init__('car_start_gateway')
        self.declare_parameter('simulation_mode', True)
        self.gate = StartGate()
        self.publisher = self.create_publisher(
            String, '/uav_mission/events/start', 10)
        self.create_subscription(
            String, '/uav_mission/sim/car_start', self._start, 10)

    def _start(self, msg):
        if not self.get_parameter('simulation_mode').value:
            self.get_logger().warning('simulation start ignored outside simulation')
            return
        try:
            request = json.loads(msg.data)
            task_id = str(request['task_id'])
            valid = bool(request.get('valid', True))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning('malformed simulated start ignored')
            return
        if not self.gate.accept(task_id, valid):
            self.get_logger().warning(
                'duplicate/invalid task_id ignored: %s' % task_id)
            return
        event = {
            'task_id': task_id,
            'stamp_ns': self.get_clock().now().nanoseconds,
            'valid': True,
        }
        output = String()
        output.data = json.dumps(event, separators=(',', ':'))
        self.publisher.publish(output)
        self.get_logger().info('accepted task_id=%s' % task_id)


def main(args=None):
    """Run the car start gateway."""
    rclpy.init(args=args)
    node = CarStartGateway()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
