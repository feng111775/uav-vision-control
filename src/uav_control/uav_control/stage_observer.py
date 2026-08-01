"""Stop a staged SITL launch after the real controller reaches a target."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .stage_catalog import validate_stage_target


class StageObserver(Node):
    def __init__(self):
        super().__init__('stage_observer')
        self.declare_parameter('target_stage', 'WAIT_PX4')
        self.declare_parameter('timeout_seconds', 60.0)
        self.target = validate_stage_target(
            self.get_parameter('target_stage').value)
        self.state = 'UNKNOWN'
        self.started = self.get_clock().now()
        self.exit_code = None
        self.create_subscription(
            String, '/uav/mission/state', self._state, 10)
        self.create_timer(0.1, self._tick)

    def _state(self, msg):
        self.state = msg.data
        if self.state == self.target:
            self.get_logger().info('Reached requested stage: ' + self.target)
            self.exit_code = 0
            rclpy.shutdown()
        elif self.state in ('DATA_TIMEOUT', 'FAILSAFE') and (
                self.target not in ('DATA_TIMEOUT', 'FAILSAFE')):
            self.get_logger().error(
                'Stopped in %s before %s' % (self.state, self.target))
            self.exit_code = 1
            rclpy.shutdown()

    def _tick(self):
        elapsed = (
            self.get_clock().now() - self.started).nanoseconds * 1e-9
        if elapsed > self.get_parameter('timeout_seconds').value:
            self.get_logger().error(
                'Timeout in %s while waiting for %s' %
                (self.state, self.target))
            self.exit_code = 1
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = StageObserver()
    try:
        rclpy.spin(node)
    finally:
        code = node.exit_code if node.exit_code is not None else 1
        node.destroy_node()
    raise SystemExit(code)
