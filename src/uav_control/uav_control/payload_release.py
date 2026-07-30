"""Dry-run one-shot payload release node."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .stage4c_core import PayloadGate


class PayloadRelease(Node):
    """Reject unsafe/repeated requests and never access GPIO in dry-run."""

    def __init__(self):
        super().__init__('payload_release')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('simulation_mode', True)
        self.gate = PayloadGate(bool(self.get_parameter('dry_run').value))
        self.publisher = self.create_publisher(
            String, '/uav_mission/payload/result', 10)
        self.create_subscription(
            String, '/uav_mission/payload/request', self._request, 10)

    def _request(self, msg):
        try:
            request = json.loads(msg.data)
            task_id = str(request['task_id'])
            release_id = str(request['release_id'])
            allowed = bool(request.get('allowed', False))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning('malformed release request rejected')
            return
        status = self.gate.request(task_id, release_id, allowed)
        if status == 'SUCCESS':
            self.get_logger().info(
                'dry-run: simulated servo rotation 90 degrees')
        else:
            self.get_logger().warning(
                'release rejected task=%s release=%s' %
                (task_id, release_id))
        output = String()
        output.data = json.dumps({
            'task_id': task_id,
            'release_id': release_id,
            'stamp_ns': self.get_clock().now().nanoseconds,
            'status': status,
        }, separators=(',', ':'))
        self.publisher.publish(output)


def main(args=None):
    """Run the dry-run payload node."""
    rclpy.init(args=args)
    node = PayloadRelease()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
