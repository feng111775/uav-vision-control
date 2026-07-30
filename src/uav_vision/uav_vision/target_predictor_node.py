"""Constant-velocity tracking and prediction for filtered D-task targets."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .d_task_schema import CENTER_X, CENTER_Y, VALID
from .d_task_schema import invalid_tracked, validate_detection


class TargetPredictor:
    def __init__(self, prediction_horizon=0.08, max_velocity=800.0,
                 max_jump=80.0, loss_timeout=0.3):
        if prediction_horizon < 0.0 or max_velocity <= 0.0:
            raise ValueError('prediction parameters are invalid')
        if max_jump <= 0.0 or loss_timeout < 0.0:
            raise ValueError('jump/timeout parameters are invalid')
        self.prediction_horizon = float(prediction_horizon)
        self.max_velocity = float(max_velocity)
        self.max_jump = float(max_jump)
        self.loss_timeout = float(loss_timeout)
        self.last_detection = None
        self.last_time = None
        self.velocity_x = 0.0
        self.velocity_y = 0.0

    def reset(self):
        self.last_detection = None
        self.last_time = None
        self.velocity_x = 0.0
        self.velocity_y = 0.0

    def process(self, values, now):
        data = validate_detection(values)
        timestamp = float(now)
        if not math.isfinite(timestamp):
            raise ValueError('timestamp must be finite')
        if data[VALID] != 1.0:
            return self._lost(timestamp)
        if self.last_detection is not None and timestamp > self.last_time:
            dt = timestamp - self.last_time
            dx = data[CENTER_X] - self.last_detection[CENTER_X]
            dy = data[CENTER_Y] - self.last_detection[CENTER_Y]
            distance = math.hypot(dx, dy)
            if distance > self.max_jump:
                scale = self.max_jump / distance
                dx *= scale
                dy *= scale
                data[CENTER_X] = self.last_detection[CENTER_X] + dx
                data[CENTER_Y] = self.last_detection[CENTER_Y] + dy
            self.velocity_x = max(-self.max_velocity,
                                  min(self.max_velocity, dx / dt))
            self.velocity_y = max(-self.max_velocity,
                                  min(self.max_velocity, dy / dt))
        self.last_detection = list(data)
        self.last_time = timestamp
        return self._make_output(data, 0.0)

    def _lost(self, now):
        if self.last_detection is None or now < self.last_time:
            return invalid_tracked()
        age = now - self.last_time
        if age > self.loss_timeout:
            return invalid_tracked()
        held = list(self.last_detection)
        held[CENTER_X] = max(0.0, held[CENTER_X] + self.velocity_x * age)
        held[CENTER_Y] = max(0.0, held[CENTER_Y] + self.velocity_y * age)
        return self._make_output(held, age)

    def _make_output(self, data, age):
        predicted_x = max(
            0.0, data[CENTER_X] + self.velocity_x * self.prediction_horizon)
        predicted_y = max(
            0.0, data[CENTER_Y] + self.velocity_y * self.prediction_horizon)
        return list(data) + [self.velocity_x, self.velocity_y,
                             predicted_x, predicted_y, age * 1000.0]


class TargetPredictorNode(Node):
    def __init__(self):
        super().__init__('target_predictor_node')
        defaults = {
            'prediction_horizon': 0.08, 'max_velocity_px_s': 800.0,
            'max_jump_px': 80.0, 'loss_timeout_sec': 0.3,
            'input_timeout_sec': 0.3,
            'filtered_detection_topic': '/vision/h7/filtered_detection',
            'status_topic': '/vision/h7/status',
            'tracked_topic': '/vision/target/tracked'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.predictor = TargetPredictor(
            self.get_parameter('prediction_horizon').value,
            self.get_parameter('max_velocity_px_s').value,
            self.get_parameter('max_jump_px').value,
            self.get_parameter('loss_timeout_sec').value)
        self.publisher = self.create_publisher(
            Float32MultiArray, self.get_parameter('tracked_topic').value, 10)
        self.input_timeout = self.get_parameter('input_timeout_sec').value
        self.last_input_time = None
        self.timeout_published = False
        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('filtered_detection_topic').value,
            self._callback, 10)
        self.status_subscription = self.create_subscription(
            String, self.get_parameter('status_topic').value,
            self._status_callback, 10)
        self.create_timer(0.05, self._check_timeout)

    def _callback(self, message):
        self.last_input_time = time.monotonic()
        self.timeout_published = False
        now = self.get_clock().now().nanoseconds / 1e9
        try:
            data = self.predictor.process(message.data, now)
        except (TypeError, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        output = Float32MultiArray()
        output.data = data
        self.publisher.publish(output)

    def _publish_invalid(self):
        output = Float32MultiArray()
        output.data = invalid_tracked()
        self.publisher.publish(output)

    def _check_timeout(self):
        if self.last_input_time is None or self.timeout_published:
            return
        if time.monotonic() - self.last_input_time <= self.input_timeout:
            return
        self.predictor.reset()
        self._publish_invalid()
        self.timeout_published = True

    def _status_callback(self, message):
        if message.data not in ('DISCONNECTED', 'STALE'):
            return
        self.predictor.reset()
        self._publish_invalid()
        self.timeout_published = True


def main(args=None):
    rclpy.init(args=args)
    node = TargetPredictorNode()
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
