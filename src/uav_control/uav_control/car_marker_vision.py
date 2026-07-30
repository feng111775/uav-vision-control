"""Publish a timestamped marker contract from simulation input."""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


def normalize_marker_error(error_x, error_y, normalized, width, height):
    """Validate and normalize detector errors without camera access."""
    values = (float(error_x), float(error_y))
    if not all(math.isfinite(value) for value in values):
        raise ValueError('marker error must be finite')
    if not normalized:
        if int(width) <= 0 or int(height) <= 0:
            raise ValueError('pixel errors require valid image dimensions')
        values = (values[0] / (0.5 * int(width)),
                  values[1] / (0.5 * int(height)))
    if any(abs(value) > 2.0 for value in values):
        raise ValueError('marker error outside accepted normalized range')
    return values


class CarMarkerVision(Node):
    """Contract adapter only; it opens no camera and controls no vehicle."""

    def __init__(self):
        super().__init__('car_marker_vision')
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('real_vision_enabled', False)
        self.declare_parameter('calibration_valid', False)
        self.declare_parameter('image_width_px', 0)
        self.declare_parameter('image_height_px', 0)
        self.declare_parameter('vision_error_normalized', True)
        simulation = bool(self.get_parameter('simulation_mode').value)
        real_enabled = bool(self.get_parameter('real_vision_enabled').value)
        calibrated = bool(self.get_parameter('calibration_valid').value)
        width = int(self.get_parameter('image_width_px').value)
        height = int(self.get_parameter('image_height_px').value)
        if real_enabled:
            if simulation:
                raise ValueError('real vision cannot be enabled in simulation')
            if not calibrated or width <= 0 or height <= 0:
                raise ValueError(
                    'real vision requires explicit valid calibration and size')
            raise ValueError(
                'real camera/detector adapter is not implemented')
        if simulation and (not calibrated or width <= 0 or height <= 0):
            raise ValueError(
                'simulation vision requires explicit calibration and size')
        self.width = width
        self.height = height
        self.normalized = bool(
            self.get_parameter('vision_error_normalized').value)
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
        if not 0.0 <= float(values[1]) <= 1.0:
            self.get_logger().warning('invalid marker confidence ignored')
            return
        try:
            error_x, error_y = normalize_marker_error(
                values[2], values[3], self.normalized,
                self.width, self.height)
        except ValueError:
            self.get_logger().warning('out-of-range marker error ignored')
            return
        optional = values[4:] + [None] * 4
        observation = {
            'detected': bool(values[0]),
            'stamp_ns': self.get_clock().now().nanoseconds,
            'confidence': float(values[1]),
            'error_x': error_x,
            'error_y': error_y,
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
