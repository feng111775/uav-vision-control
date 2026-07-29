"""Render D-task metadata onto a camera-independent debug canvas."""

import math

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from . import d_task_schema as schema


class VisionDashboardNode(Node):
    def __init__(self):
        super().__init__('vision_dashboard_node')
        defaults = {'tracked_topic': '/vision/target/tracked',
                    'landing_error_topic': '/vision/landing_error',
                    'canvas_topic': '/vision/debug/target_canvas',
                    'status_topic': '/vision/debug/status',
                    'detector_status_topic': '/vision/h7/status',
                    'enable_local_window': False, 'input_timeout_sec': 0.4,
                    'publish_rate_hz': 10.0}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.bridge = CvBridge()
        self.tracked = None
        self.error = None
        self.last_input_time = None
        self.detector_status = 'LOST'
        self.last_detector_status_time = None
        self.image_publisher = self.create_publisher(
            Image, self.get_parameter('canvas_topic').value, 10)
        self.status_publisher = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(
            Float32MultiArray, self.get_parameter('tracked_topic').value,
            self._tracked_callback, 10)
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter('landing_error_topic').value,
            self._error_callback, 10)
        self.create_subscription(
            String, self.get_parameter('detector_status_topic').value,
            self._detector_status_callback, 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._render)

    def _tracked_callback(self, message):
        try:
            self.tracked = schema.validate_tracked(message.data)
            self.last_input_time = self.get_clock().now()
        except (TypeError, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)

    def _error_callback(self, message):
        try:
            self.error = schema.validate_landing_error(message.data)
        except (TypeError, ValueError):
            self.error = None

    def _detector_status_callback(self, message):
        if message.data in ('TRACKING', 'LOST', 'CROSS_INVALID',
                            'DETECT_ERROR'):
            self.detector_status = message.data
            self.last_detector_status_time = self.get_clock().now()

    def _render(self):
        canvas = np.zeros((480, 640, 3), dtype=np.uint8)
        center = (320, 240)
        cv2.drawMarker(canvas, center, (180, 180, 180), cv2.MARKER_CROSS, 24, 2)
        now = self.get_clock().now()
        timeout = float(self.get_parameter('input_timeout_sec').value)
        fresh = (self.tracked is not None and self.last_input_time is not None
                 and (now - self.last_input_time).nanoseconds / 1e9 <= timeout)
        valid = fresh and self.tracked[schema.VALID] == 1.0
        detector_fresh = (
            self.last_detector_status_time is not None and
            (now - self.last_detector_status_time).nanoseconds / 1e9 <= timeout)
        status = 'TRACKING' if valid else (
            self.detector_status if detector_fresh else 'LOST')
        if valid:
            data = self.tracked
            target = (round(data[schema.CENTER_X] * 2),
                      round(data[schema.CENTER_Y] * 2))
            predicted = (round(data[schema.PREDICTED_CENTER_X] * 2),
                         round(data[schema.PREDICTED_CENTER_Y] * 2))
            outer = round(data[schema.OUTER_DIAMETER])
            inner = round(data[schema.INNER_DIAMETER])
            cv2.circle(canvas, target, outer, (0, 200, 255), 2)
            cv2.circle(canvas, target, inner, (0, 255, 100), 2)
            cv2.circle(canvas, target, 5, (0, 0, 255), -1)
            cv2.circle(canvas, predicted, 6, (255, 0, 255), 2)
            cv2.line(canvas, center, target, (255, 180, 0), 2)
            length = max(20, inner)
            angle = data[schema.ANGLE]
            direction = (round(target[0] + length * math.cos(angle)),
                         round(target[1] + length * math.sin(angle)))
            cv2.line(canvas, target, direction, (255, 255, 0), 2)
            lines = [
                'valid=1 state=TRACKING',
                'center=(%.1f,%.1f) diameter=(%.1f,%.1f)' % (
                    data[schema.CENTER_X], data[schema.CENTER_Y],
                    data[schema.OUTER_DIAMETER], data[schema.INNER_DIAMETER]),
                'angle=%.3f confidence=%.1f age=%.1fms' % (
                    angle, data[schema.CONFIDENCE], data[schema.TARGET_AGE_MS]),
                'predicted=(%.1f,%.1f) velocity=(%.1f,%.1f)px/s' % (
                    data[schema.PREDICTED_CENTER_X],
                    data[schema.PREDICTED_CENTER_Y], data[schema.VELOCITY_X],
                    data[schema.VELOCITY_Y])]
        else:
            lines = ['valid=0 state=%s' % status,
                     'center=(0,0) diameter=(0,0)',
                     'angle=0 confidence=0 age=0ms',
                     'predicted=(0,0) velocity=(0,0)px/s']
            cv2.putText(canvas, status, (220, 260), cv2.FONT_HERSHEY_SIMPLEX,
                        1.2, (0, 0, 255), 3)
        for index, line in enumerate(lines):
            cv2.putText(canvas, line, (12, 28 + 26 * index),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (240, 240, 240), 1)
        image = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        image.header.stamp = now.to_msg()
        image.header.frame_id = 'd_task_debug_canvas'
        self.image_publisher.publish(image)
        message = String()
        message.data = '; '.join(lines) + '; visual_state=' + status
        self.status_publisher.publish(message)
        if self.get_parameter('enable_local_window').value:
            cv2.imshow('D-task target dashboard', canvas)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.get_parameter('enable_local_window').value:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionDashboardNode()
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
