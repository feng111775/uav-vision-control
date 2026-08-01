"""Convert tracked target pixels into normalized D-task landing errors."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from .d_task_schema import ANGLE, CENTER_X, CENTER_Y, CONFIDENCE
from .d_task_schema import TARGET_AGE_MS, VALID, invalid_landing_error
from .d_task_schema import validate_tracked


class LandingErrorCalculator:
    def __init__(self, image_center_x=160.0, image_center_y=120.0,
                 input_timeout_ms=300.0):
        if image_center_x <= 0.0 or image_center_y <= 0.0:
            raise ValueError('image center must be positive')
        self.center_x = float(image_center_x)
        self.center_y = float(image_center_y)
        self.input_timeout_ms = float(input_timeout_ms)

    def calculate(self, values):
        data = validate_tracked(values)
        if data[VALID] != 1.0 or data[TARGET_AGE_MS] > self.input_timeout_ms:
            return invalid_landing_error()
        error_x = data[CENTER_X] - self.center_x
        error_y = data[CENTER_Y] - self.center_y
        return [1.0, error_x / self.center_x, error_y / self.center_y,
                error_x, error_y, data[ANGLE], data[CONFIDENCE],
                data[TARGET_AGE_MS]]


class LandingErrorNode(Node):
    def __init__(self):
        super().__init__('landing_error_node')
        defaults = {'image_center_x': 160.0, 'image_center_y': 120.0,
                    'input_timeout_ms': 300.0,
                    'tracked_topic': '/vision/target/tracked',
                    'landing_error_topic': '/vision/landing_error'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.calculator = LandingErrorCalculator(
            self.get_parameter('image_center_x').value,
            self.get_parameter('image_center_y').value,
            self.get_parameter('input_timeout_ms').value)
        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('landing_error_topic').value, 10)
        self.health_publisher = self.create_publisher(Bool, '/vision/health', 10)
        self.create_timer(0.5, lambda: self.health_publisher.publish(Bool(data=True)))
        self.subscription = self.create_subscription(
            Float32MultiArray, self.get_parameter('tracked_topic').value,
            self._callback, 10)

    def _callback(self, message):
        try:
            data = self.calculator.calculate(message.data)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            data = invalid_landing_error()
        output = Float32MultiArray()
        output.data = data
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = LandingErrorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
