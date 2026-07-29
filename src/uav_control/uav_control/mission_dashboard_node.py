"""Read-only mission telemetry dashboard."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from .mission_schema import TELEMETRY, TELEMETRY_LENGTH


class MissionDashboardNode(Node):
    def __init__(self):
        super().__init__('mission_dashboard_node')
        self.bridge = CvBridge()
        self.telemetry = [0.0] * TELEMETRY_LENGTH
        self.state = 'NO_DATA'
        self.event = 'NONE'
        self.pub = self.create_publisher(Image, '/uav/mission/debug_canvas', 10)
        self.create_subscription(Float32MultiArray, '/uav/mission/telemetry', self._telemetry, 10)
        self.create_subscription(
            String,
            '/uav/mission/state',
            lambda m: setattr(
                self,
                'state',
                m.data),
            10)
        self.create_subscription(
            String,
            '/uav/mission/event',
            lambda m: setattr(
                self,
                'event',
                m.data),
            10)
        self.create_timer(0.1, self._render)

    def _telemetry(self, msg):
        if len(msg.data) == TELEMETRY_LENGTH:
            self.telemetry = list(msg.data)

    def _render(self):
        t = self.telemetry
        image = np.zeros((600, 900, 3), np.uint8)
        mode = {1: 'drop', 2: 'dynamic_land', 3: 'hover_test'}.get(
            round(t[TELEMETRY['mission_mode']]), 'unknown')
        lines = [f'D-TASK mission={mode} state={self.state}',
                 'elapsed=%.1f/90.0 s  event/risk=%s' %
                 (t[TELEMETRY['elapsed_seconds']], self.event),
                 'position NED=(%.2f, %.2f, %.2f)  H distance=%.2f m' %
                 (t[4], t[5], t[6], t[TELEMETRY['h_distance']]),
                 'velocity=(%.2f, %.2f, %.2f) heading=%.2f' % (t[7], t[8], t[9], t[10]),
                 'armed=%d offboard=%d failsafe=%d car_progress=%d' % (t[16], t[17], t[18], t[15]),
                 'target=%d confidence=%.1f error=(%.3f, %.3f)' % (t[11], t[14], t[12], t[13]),
                 'payload/event=%s touchdown=%d ack_attempt=%d' % (self.event, t[20], t[19]),
                 'H point is captured from first stable valid local position.',
                 'Car markers: A -> B -> C -> D -> A (read-only progress)']
        for i, line in enumerate(lines):
            cv2.putText(image, line, (25, 45 + i * 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, (230, 230, 230), 2)
        cv2.line(image, (80, 540), (820, 540), (80, 150, 255), 2)
        x = int(80 + min(5, max(0, t[15])) / 5 * 740)
        cv2.circle(image, (x, 540), 12, (0, 255, 100), -1)
        msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mission_dashboard'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
