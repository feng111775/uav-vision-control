"""SITL-only mock using the exact D-task controller topic contracts."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8


class DTaskMockNode(Node):
    def __init__(self):
        super().__init__('d_task_mock_node')
        self.declare_parameter('simulation_mode', False)
        if not self.get_parameter('simulation_mode').value:
            raise RuntimeError('D-task mock is forbidden outside simulation')
        self.state = 'WAIT_PX4'
        self.release_seen = False
        self.start_pub = self.create_publisher(
            Bool, '/car/mission_start', 10)
        self.progress_pub = self.create_publisher(
            UInt8, '/car/progress', 10)
        self.error_pub = self.create_publisher(
            Float32MultiArray, '/vision/landing_error', 10)
        self.tracked_pub = self.create_publisher(
            Float32MultiArray, '/vision/target/tracked', 10)
        self.ack_pub = self.create_publisher(
            Bool, '/uav/payload/release_ack', 10)
        self.touchdown_pub = self.create_publisher(
            Bool, '/uav/touchdown_sensor', 10)
        self.create_subscription(
            String, '/uav/mission/state', self._state, 10)
        self.create_subscription(
            Bool, '/uav/payload/release', self._release, 10)
        self.create_timer(0.05, self._tick)

    def _state(self, msg):
        self.state = msg.data

    def _release(self, msg):
        self.release_seen = self.release_seen or bool(msg.data)

    def _tick(self):
        start = Bool()
        start.data = True
        self.start_pub.publish(start)
        progress = UInt8()
        progress.data = 1
        self.progress_pub.publish(progress)
        error = Float32MultiArray()
        error.data = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 90.0, 0.0]
        self.error_pub.publish(error)
        tracked = Float32MultiArray()
        tracked.data = [
            1.0, 160.0, 120.0, 50.0, 30.0, 0.0,
            90.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.tracked_pub.publish(tracked)
        ack = Bool()
        ack.data = self.release_seen
        self.ack_pub.publish(ack)
        touchdown = Bool()
        touchdown.data = self.state in (
            'DYNAMIC_DESCENT_HIGH', 'DYNAMIC_DESCENT_NEAR',
            'TOUCHDOWN_CHECK')
        self.touchdown_pub.publish(touchdown)


def main(args=None):
    rclpy.init(args=args)
    node = DTaskMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
