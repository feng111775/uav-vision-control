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
            'mission_state_topic': '/control/qr_mission_state',
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
        self.state_subscription = self.create_subscription(
            String, value('mission_state_topic'), self.state_callback, 10)
        self.last_confirmed_count = 0
        self.get_logger().info(
            'QR detector ready: backend=%s target=%d %s' % (
                value('detector_backend'), value('target_qr_id'),
                self.detector.fallback_reason))

    def state_callback(self, message):
        """Accept scan-point gating and position from the flight controller."""
        try:
            data = json.loads(message.data)
            self.inventory.set_scan_context(
                data['scan_index'], data.get('hold', False),
                data.get('scan_position', []), data.get('retry_count', 0),
                data.get('state', 'QR_SCAN_MOVE'))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            # Backward-compatible plain task states cannot authorize a scan.
            pass

    def callback(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            image = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        except Exception as error:
            self.diagnostics_pub.publish(String(
                data=json.dumps({'ok': False, 'error': str(error)})))
            return
        observations = self.detector.detect(image, now)
        self.inventory.image_size = (image.shape[1], image.shape[0])
        self.inventory.update(observations, now)
        if len(self.inventory.records) > self.last_confirmed_count:
            record = self.inventory.records[
                list(self.inventory.records)[-1]]
            self.get_logger().info(
                '[QR SCAN] confirmed %d/24: QR %d' % (
                    len(self.inventory.records), record['qr_id']))
            self.last_confirmed_count = len(self.inventory.records)
            if self.inventory.complete(now):
                self.get_logger().info(
                    '[QR SCAN] inventory complete: 24/24')
                self.get_logger().info(
                    '[QR SCAN] target selected: QR %d' %
                    self.inventory.target_qr_id)
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
            import cv2
            for obs in observations:
                x, y, width, height = obs.bbox
                cv2.rectangle(image, (x, y), (x + width, y + height),
                              (0, 255, 0), 2)
                cv2.putText(image, 'decoded QR %d' % obs.qr_id,
                            (x, max(15, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            expected = self.inventory.expected_qr_id
            lines = [
                'SCAN %d/24  EXPECT QR %s' % (
                    min(self.inventory.scan_index + 1, 24),
                    '-' if expected is None else expected),
                '%s  confirmed=%d  target=%d' % (
                    'CONFIRMED' if self.inventory.status in (
                        'QR_SCAN_NEXT', 'QR_INVENTORY_COMPLETE') else
                    'RETRY %d' % self.inventory.retry_count,
                    len(self.inventory.records), self.inventory.target_qr_id)]
            for row, text_value in enumerate(lines):
                cv2.putText(image, text_value, (12, 28 + row * 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                            (0, 220, 255), 2)
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
