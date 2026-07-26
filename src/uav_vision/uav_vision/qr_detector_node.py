"""ROS 2 QR detector, inventory, target and compatibility publisher."""

import json

from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, String

from .qr_core import HybridQRDetector, LaserAlignment, QRInventory, load_layout


class QRDetectorNode(Node):
    """Process front images without ever publishing commands to PX4."""

    def __init__(self):
        super().__init__('qr_detector_node')
        defaults = {
            'image_topic': '/camera/front/image_raw',
            'detector_backend': 'hybrid', 'model_path': '',
            'layout_path': '', 'target_qr_id': 1,
            'inventory_mode': 'target', 'confidence_threshold': 0.35,
            'confirm_frames': 3, 'qr_timeout': 30.0,
            'image_timeout': 0.5, 'laser_alignment_threshold': 12.0,
            'laser_confirm_frames': 5, 'visualization': True,
            'detection_topic': '/vision/qr/detection',
            'confirmed_topic': '/vision/qr/confirmed',
            'inventory_topic': '/vision/qr/inventory',
            'target_topic': '/vision/qr/target',
            'laser_topic': '/vision/qr/laser_aligned',
            'diagnostics_topic': '/vision/qr/diagnostics',
            'debug_image_topic': '/vision/qr/debug_image'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.detector = HybridQRDetector(
            value('detector_backend'), value('model_path'),
            value('confidence_threshold'))
        layout = load_layout(value('layout_path')) if value(
            'layout_path') else {}
        self.inventory = QRInventory(
            layout, value('confirm_frames'), value('qr_timeout'),
            value('image_timeout'), value('target_qr_id'),
            value('inventory_mode'))
        self.laser = LaserAlignment(
            value('laser_alignment_threshold'),
            value('laser_confirm_frames'))
        self.bridge = CvBridge()
        self.visualization = value('visualization')
        self.detection_pub = self.create_publisher(
            Float32MultiArray, value('detection_topic'), 10)
        self.confirmed_pub = self.create_publisher(
            String, value('confirmed_topic'), 10)
        self.inventory_pub = self.create_publisher(
            String, value('inventory_topic'), 10)
        self.target_pub = self.create_publisher(
            Float32MultiArray, value('target_topic'), 10)
        self.laser_pub = self.create_publisher(
            Bool, value('laser_topic'), 10)
        self.diagnostics_pub = self.create_publisher(
            String, value('diagnostics_topic'), 10)
        self.debug_pub = self.create_publisher(
            Image, value('debug_image_topic'), qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Image, value('image_topic'), self.callback,
            qos_profile_sensor_data)
        self.get_logger().info(
            'QR detector ready: backend=%s target=%d %s' % (
                value('detector_backend'), value('target_qr_id'),
                self.detector.fallback_reason))

    def callback(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except Exception as error:
            self.diagnostics_pub.publish(String(
                data=json.dumps({'ok': False, 'error': str(error)})))
            return
        observations = self.detector.detect(image, now)
        self.inventory.update(observations, now)
        for obs in observations:
            output = Float32MultiArray(data=[
                float(obs.qr_id)] + obs.detection_array())
            self.detection_pub.publish(output)
            if obs.qr_id in self.inventory.records:
                self.confirmed_pub.publish(String(data=json.dumps(
                    self.inventory.records[obs.qr_id],
                    ensure_ascii=False)))
            if obs.qr_id == self.inventory.target_qr_id:
                compatibility = Float32MultiArray(
                    data=obs.detection_array())
                self.target_pub.publish(compatibility)
                error_x = obs.center[0] - image.shape[1] / 2.0
                error_y = obs.center[1] - image.shape[0] / 2.0
                self.laser_pub.publish(Bool(
                    data=self.laser.update(error_x, error_y)))
        self.inventory_pub.publish(String(data=self.inventory.to_json()))
        self.diagnostics_pub.publish(String(data=json.dumps({
            'ok': True, 'stamp_ns': int(message.header.stamp.sec * 1e9 +
                                        message.header.stamp.nanosec),
            'detections': len(observations),
            'inventory_count': len(self.inventory.records),
            'model_available': self.detector.learned.available,
            'fallback': self.detector.fallback_reason})))
        if self.visualization and self.debug_pub.get_subscription_count():
            for obs in observations:
                x, y, width, height = obs.bbox
                import cv2
                cv2.rectangle(image, (x, y), (x + width, y + height),
                              (0, 255, 0), 2)
                cv2.putText(image, str(obs.qr_id), (x, max(15, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            output = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            output.header = message.header
            self.debug_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = QRDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
