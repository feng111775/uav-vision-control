# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Bench-only car simulator; never publishes motor commands."""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8


class CarLinkSimulator(Node):  # noqa: D101
    def __init__(self) -> None:  # noqa: D107
        super().__init__("car_link_simulator_node")
        self.declare_parameter("period_seconds", 2.0)
        self.start_pub = self.create_publisher(Bool, "/car/mission_start", 10)
        self.progress_pub = self.create_publisher(UInt8, "/car/progress", 10)
        self.started = time.monotonic()
        self.create_timer(0.2, self._tick)

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started
        self.start_pub.publish(Bool(data=elapsed >= 0.5))
        period = float(self.get_parameter("period_seconds").value)
        self.progress_pub.publish(UInt8(data=min(5, int(elapsed / period))))


def main(args=None) -> None:  # noqa: D103
    rclpy.init(args=args)
    node = CarLinkSimulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
