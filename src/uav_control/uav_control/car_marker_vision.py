"""Publish a timestamped marker contract from simulation input."""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class CarMarkerVision(Node):
    """Contract adapter only; it opens no camera and controls no vehicle."""

    def __init__(self):
        super().__init__('car_marker_vision')
        self.declare_parameter('simulation_mode', True)
        self.publisher = self.create_publisher(
            String, '/uav_mission/vision/marker', 10)
        self.create_subscription(
            Float32MultiArray, '/uav_mission/sim/marker', self._marker, 10)

    def _marker(self, msg):
        if not self.get_parameter('simulation_mode').value:
            return
        values = list(msg.data)
        if len(values) < 4 or not all(math.isfinite(v) for v in values):
            self.get_logger().warning('invalid simulated marker ignored')
            return
        optional = values[4:] + [None] * 4
        observation = {
            'detected': bool(values[0]),
            'stamp_ns': self.get_clock().now().nanoseconds,
            'confidence': float(values[1]),
            'error_x': float(values[2]),
            'error_y': float(values[3]),
            'relative_x': optional[0],
            'relative_y': optional[1],
            'velocity_x': optional[2],
            'velocity_y': optional[3],
        }
        output = String()
        output.data = json.dumps(observation, separators=(',', ':'))
        self.publisher.publish(output)


def main(args=None):
    """Run the marker contract adapter."""
    rclpy.init(args=args)
    node = CarMarkerVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
