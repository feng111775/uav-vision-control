"""Detect the largest red Gazebo target and publish H7-compatible data."""

import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


INVALID_DETECTION = [0.0] * 7


class RedTargetDetector:
    """ROS-independent red target image detector."""

    def __init__(
            self, min_area=100.0, confidence=90.0, morphology_kernel=5,
            red1_h_min=0, red1_h_max=10, red2_h_min=170, red2_h_max=179,
            saturation_min=100, value_min=70):
        numeric = (
            min_area, confidence, morphology_kernel, red1_h_min, red1_h_max,
            red2_h_min, red2_h_max, saturation_min, value_min)
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError('detector parameters must be finite')
        if min_area < 0.0:
            raise ValueError('min_area cannot be negative')
        if not 0.0 <= confidence <= 100.0:
            raise ValueError('confidence must be in [0, 100]')
        if not (0 <= red1_h_min <= red1_h_max <= 179):
            raise ValueError('first hue range must be inside [0, 179]')
        if not (0 <= red2_h_min <= red2_h_max <= 179):
            raise ValueError('second hue range must be inside [0, 179]')
        if not (0 <= saturation_min <= 255 and 0 <= value_min <= 255):
            raise ValueError('S/V thresholds must be inside [0, 255]')

        self.min_area = float(min_area)
        self.confidence = float(confidence)
        kernel_size = max(1, int(morphology_kernel))
        if kernel_size % 2 == 0:
            kernel_size += 1
        self.kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        self.lower_red1 = np.array(
            [red1_h_min, saturation_min, value_min], dtype=np.uint8)
        self.upper_red1 = np.array(
            [red1_h_max, 255, 255], dtype=np.uint8)
        self.lower_red2 = np.array(
            [red2_h_min, saturation_min, value_min], dtype=np.uint8)
        self.upper_red2 = np.array(
            [red2_h_max, 255, 255], dtype=np.uint8)

    def detect(self, image):
        """Return a seven-value detection, debug image, and binary mask."""
        if not isinstance(image, np.ndarray) or image.size == 0:
            return list(INVALID_DETECTION), None, None
        if image.ndim != 3 or image.shape[2] not in (3, 4):
            return list(INVALID_DETECTION), None, None

        bgr = image[:, :, :3]
        debug_image = bgr.copy()
        try:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        except cv2.error:
            return list(INVALID_DETECTION), debug_image, None
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.lower_red1, self.upper_red1),
            cv2.inRange(hsv, self.lower_red2, self.upper_red2),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        height, width = bgr.shape[:2]
        cv2.drawMarker(
            debug_image, (width // 2, height // 2), (255, 255, 0),
            cv2.MARKER_CROSS, 16, 1)
        candidates = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self.min_area
        ]
        if not candidates:
            return list(INVALID_DETECTION), debug_image, mask

        contour = max(candidates, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        x, y, box_width, box_height = cv2.boundingRect(contour)
        center_x = min(float(width - 1), max(0.0, x + box_width / 2.0))
        center_y = min(float(height - 1), max(0.0, y + box_height / 2.0))
        result = [
            1.0, center_x, center_y, float(box_width), float(box_height),
            area, self.confidence,
        ]
        if not all(math.isfinite(value) for value in result):
            return list(INVALID_DETECTION), debug_image, mask

        cv2.drawContours(debug_image, [contour], -1, (0, 255, 0), 2)
        cv2.rectangle(
            debug_image, (x, y), (x + box_width - 1, y + box_height - 1),
            (255, 0, 0), 1)
        cv2.circle(
            debug_image, (round(center_x), round(center_y)), 4,
            (0, 255, 255), -1)
        return result, debug_image, mask


class GazeboRedTargetDetectorNode(Node):
    """Convert Gazebo RGB images to the existing H7 detection protocol."""

    def __init__(self):
        super().__init__('gazebo_red_target_detector_node')
        defaults = {
            'image_topic': '/camera/down/image_raw',
            'min_area': 100.0,
            'confidence': 90.0,
            'morphology_kernel': 5,
            'red1_h_min': 0,
            'red1_h_max': 10,
            'red2_h_min': 170,
            'red2_h_max': 179,
            'saturation_min': 100,
            'value_min': 70,
            'debug': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.detector = RedTargetDetector(**{
            name: self.get_parameter(name).value
            for name in defaults if name not in ('image_topic', 'debug')
        })
        self.debug = bool(self.get_parameter('debug').value)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Float32MultiArray, '/vision/h7/detection', 10)
        self.debug_publisher = self.create_publisher(
            Image, '/vision/gazebo/debug_image', 10)
        self.subscription = self.create_subscription(
            Image, self.get_parameter('image_topic').value,
            self.image_callback, 10)
        self.get_logger().info(
            'Gazebo red detector started; H7 bridge must remain stopped')

    def publish_detection(self, values):
        """Publish exactly seven finite float values."""
        message = Float32MultiArray()
        message.data = [float(value) for value in values]
        self.publisher.publish(message)

    def image_callback(self, message):
        """Process an image; malformed frames safely produce invalid data."""
        try:
            image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8')
            detection, debug_image, _ = self.detector.detect(image)
        except (cv2.error, TypeError, ValueError) as error:
            self.get_logger().warning(
                'Invalid camera image: %s' % error,
                throttle_duration_sec=5.0)
            detection, debug_image = list(INVALID_DETECTION), None

        self.publish_detection(detection)
        if self.debug and debug_image is not None:
            debug_message = self.bridge.cv2_to_imgmsg(
                debug_image, encoding='bgr8')
            debug_message.header = message.header
            self.debug_publisher.publish(debug_message)


def main(args=None):
    """Run the Gazebo red target detector node."""
    rclpy.init(args=args)
    node = GazeboRedTargetDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
