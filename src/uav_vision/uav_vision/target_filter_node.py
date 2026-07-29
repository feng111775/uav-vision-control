"""Confirm and periodically filter D-task H7 detections."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .d_task_schema import ANGLE, CONFIDENCE, VALID
from .d_task_schema import invalid_detection, periodic_angle_difference
from .d_task_schema import normalize_periodic_angle, validate_detection


class TargetFilter:
    def __init__(self, alpha=0.35, angle_alpha=0.35, min_confidence=50.0,
                 confirm_frames=3, lost_frames=3):
        if not 0.0 < alpha <= 1.0 or not 0.0 < angle_alpha <= 1.0:
            raise ValueError('EMA alphas must be in (0, 1]')
        if not 0.0 <= min_confidence <= 100.0:
            raise ValueError('min_confidence must be in [0, 100]')
        if confirm_frames < 1 or lost_frames < 1:
            raise ValueError('frame thresholds must be positive')
        self.alpha = float(alpha)
        self.angle_alpha = float(angle_alpha)
        self.min_confidence = float(min_confidence)
        self.confirm_frames = int(confirm_frames)
        self.lost_frames = int(lost_frames)
        self.confirm_count = 0
        self.lost_count = 0
        self.confirmed = False
        self.filtered_values = None

    def process(self, values):
        data = validate_detection(values)
        if data[VALID] != 1.0 or data[CONFIDENCE] < self.min_confidence:
            return self._miss()
        if self.filtered_values is None:
            self.filtered_values = list(data)
            self.filtered_values[ANGLE] = normalize_periodic_angle(data[ANGLE])
        else:
            for index in range(1, len(data)):
                if index == ANGLE:
                    difference = periodic_angle_difference(
                        data[ANGLE], self.filtered_values[ANGLE])
                    self.filtered_values[ANGLE] = normalize_periodic_angle(
                        self.filtered_values[ANGLE] + self.angle_alpha * difference)
                else:
                    self.filtered_values[index] += self.alpha * (
                        data[index] - self.filtered_values[index])
        self.filtered_values[VALID] = 1.0
        self.lost_count = 0
        if not self.confirmed:
            self.confirm_count += 1
            self.confirmed = self.confirm_count >= self.confirm_frames
        return list(self.filtered_values) if self.confirmed else invalid_detection()

    def _miss(self):
        if not self.confirmed:
            self.confirm_count = 0
            self.filtered_values = None
            return invalid_detection()
        self.lost_count += 1
        if self.lost_count <= self.lost_frames:
            return list(self.filtered_values)
        self.confirmed = False
        self.confirm_count = 0
        self.lost_count = 0
        self.filtered_values = None
        return invalid_detection()


class TargetFilterNode(Node):
    def __init__(self):
        super().__init__('target_filter_node')
        defaults = {
            'alpha': 0.35, 'angle_alpha': 0.35, 'min_confidence': 50.0,
            'confirm_frames': 3, 'lost_frames': 3,
            'detection_topic': '/vision/h7/detection',
            'filtered_detection_topic': '/vision/h7/filtered_detection'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.filter = TargetFilter(**{
            name: self.get_parameter(name).value for name in
            ('alpha', 'angle_alpha', 'min_confidence',
             'confirm_frames', 'lost_frames')})
        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('filtered_detection_topic').value, 10)
        self.subscription = self.create_subscription(
            Float32MultiArray, self.get_parameter('detection_topic').value,
            self._callback, 10)

    def _callback(self, message):
        try:
            data = self.filter.process(message.data)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        output = Float32MultiArray()
        output.data = data
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = TargetFilterNode()
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
