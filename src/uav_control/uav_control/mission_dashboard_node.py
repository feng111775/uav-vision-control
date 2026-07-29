"""Read-only dashboard for unified D-task mission telemetry."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from .mission_schema import (SAFETY_BLOCK_NAMES, TELEMETRY,
                             TELEMETRY_LENGTH)


class MissionDashboardNode(Node):
    """Render existing mission topics without publishing control commands."""

    def __init__(self):
        super().__init__('mission_dashboard_node')
        self.bridge = CvBridge()
        self.telemetry = [0.0] * TELEMETRY_LENGTH
        self.state = 'NO_DATA'
        self.event = 'NONE'
        self.pub = self.create_publisher(
            Image, '/uav/mission/debug_canvas', 10)
        self.create_subscription(
            Float32MultiArray, '/uav/mission/telemetry',
            self._telemetry, 10)
        self.create_subscription(
            String, '/uav/mission/state',
            lambda msg: setattr(self, 'state', msg.data), 10)
        self.create_subscription(
            String, '/uav/mission/event',
            lambda msg: setattr(self, 'event', msg.data), 10)
        self.create_timer(0.1, self._render)

    def _telemetry(self, msg):
        if len(msg.data) == TELEMETRY_LENGTH:
            self.telemetry = list(msg.data)

    def value(self, name):
        return self.telemetry[TELEMETRY[name]]

    def display_lines(self):
        """Return human-readable lines; kept pure for dashboard tests."""
        mode = {1: 'drop', 2: 'dynamic_land', 3: 'hover_test'}.get(
            round(self.value('mission_mode')), 'unknown')
        block = SAFETY_BLOCK_NAMES.get(
            round(self.value('safety_block_code')), 'UNKNOWN')
        return [
            f'D-TASK  mode={mode}  state={self.state}',
            'state %.1fs | mission %.1fs | remaining %.1fs' % (
                self.value('state_elapsed_seconds'),
                self.value('elapsed_seconds'),
                self.value('remaining_seconds')),
            'armed=%d nav=%d offboard=%d failsafe=%d PX4_fresh=%d' % (
                self.value('armed'), self.value('nav_state'),
                self.value('offboard_active'), self.value('failsafe'),
                self.value('px4_fresh')),
            'NED=(%.2f, %.2f, %.2f)m  rel-H=%.2fm  H-dist=%.2fm' % (
                self.value('x'), self.value('y'), self.value('z'),
                self.value('relative_h_height'), self.value('h_distance')),
            'velocity=(%.2f, %.2f, %.2f)m/s heading=%.2frad' % (
                self.value('vx'), self.value('vy'), self.value('vz'),
                self.value('heading')),
            'vision=%d local-age=%.0fms conf=%.1f error=(%.3f, %.3f)' % (
                self.value('target_valid'), self.value('vision_age_ms'),
                self.value('target_confidence'),
                self.value('target_error_x'),
                self.value('target_error_y')),
            'car=%d follow-before-B=%d completed-before-D=%d' % (
                self.value('car_progress'),
                self.value('formed_follow_before_b'),
                self.value('completed_before_d')),
            'payload sent=%d ack=%d command-attempt=%d' % (
                self.value('payload_sent'), self.value('payload_ack'),
                self.value('command_ack_status')),
            'touch candidate=%d confirmed=%d dwell=%.0f%%' % (
                self.value('touchdown_candidate'),
                self.value('touchdown_confirmed'),
                100.0 * self.value('dwell_progress')),
            f'safety-block={block}  event={self.event}',
        ]

    def _render(self):
        image = np.zeros((720, 1040, 3), np.uint8)
        for index, line in enumerate(self.display_lines()):
            color = (80, 220, 255) if index == 9 else (230, 230, 230)
            cv2.putText(image, line, (25, 45 + index * 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.70, color, 2)
        cv2.line(image, (90, 650), (950, 650), (80, 150, 255), 2)
        progress = min(5.0, max(0.0, self.value('car_progress')))
        x = int(90 + progress / 5.0 * 860)
        cv2.circle(image, (x, 650), 12, (0, 255, 100), -1)
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
