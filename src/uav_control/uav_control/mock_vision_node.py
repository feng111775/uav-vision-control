"""Explicit simulation-only publisher for the legacy vision adapter."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray

from .vision_contract import (make_legacy_aligned, make_legacy_malformed,
                              make_legacy_no_target)


class MockVisionNode(Node):
    """Publish deterministic legacy arrays; never opens a camera or PX4 link."""

    SCENARIOS = {'no_target', 'acquire', 'follow_converge', 'aligned',
                 'intermittent_loss', 'stale_timestamp', 'malformed',
                 'hard_loss', 'health_down'}

    def __init__(self):
        super().__init__('mock_vision_node')
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('scenario', 'no_target')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('acquire_after_seconds', 2.0)
        self.declare_parameter('vision_tracked_topic', '/vision/target/tracked')
        self.declare_parameter('vision_landing_topic', '/vision/landing_error')
        self.declare_parameter('vision_health_topic', '/vision/health_legacy_bool')
        if not bool(self.get_parameter('simulation_mode').value):
            raise ValueError('mock_vision_node requires simulation_mode=true')
        self.scenario = str(self.get_parameter('scenario').value)
        if self.scenario not in self.SCENARIOS:
            raise ValueError('unsupported mock vision scenario')
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.started = self.get_clock().now().nanoseconds / 1e9
        self.tracked_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('vision_tracked_topic').value, 10)
        self.landing_pub = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('vision_landing_topic').value, 10)
        self.health_pub = self.create_publisher(
            Bool, self.get_parameter('vision_health_topic').value, 10)
        self.create_timer(1.0 / rate, self._publish)

    def _publish(self):
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.started
        scenario = self.scenario
        health = scenario != 'health_down'
        self.health_pub.publish(Bool(data=health))
        if scenario == 'hard_loss' or (scenario == 'acquire' and
                                       elapsed < float(self.get_parameter(
                                           'acquire_after_seconds').value)):
            tracked, landing = make_legacy_no_target()
        elif scenario == 'intermittent_loss' and int(elapsed * 10) % 5 == 0:
            tracked, landing = make_legacy_no_target()
        elif scenario == 'malformed':
            tracked, landing = make_legacy_malformed()
        elif scenario == 'stale_timestamp':
            tracked, landing = make_legacy_aligned(target_age_ms=5000.0)
        elif scenario == 'aligned' or scenario == 'health_down':
            tracked, landing = make_legacy_aligned()
        elif scenario == 'follow_converge':
            error = max(0.0, 0.4 - elapsed * 0.08)
            tracked, landing = make_legacy_aligned(
                error_x=error, error_y=error)
        else:
            tracked, landing = make_legacy_no_target()
        self.tracked_pub.publish(Float32MultiArray(data=tracked))
        self.landing_pub.publish(Float32MultiArray(data=landing))


def main(args=None):
    rclpy.init(args=args)
    node = MockVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
