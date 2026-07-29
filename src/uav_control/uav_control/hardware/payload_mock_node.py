# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Bench-only payload ACK simulator."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .payload_protocol import PayloadAck, PayloadResult, encode_drop_ack


class PayloadMock(Node):  # noqa: D101
    def __init__(self) -> None:  # noqa: D107
        super().__init__("payload_mock_node")
        self.ack_pub = self.create_publisher(String, "/uav/payload/mock_ack", 10)
        self.create_subscription(
            String, "/uav/payload/mock_command", self._command, 10
        )

    def _command(self, msg: String) -> None:
        fields = msg.data.split(",")
        if len(fields) >= 3 and fields[0] == "DROP":
            self.ack_pub.publish(String(data=encode_drop_ack(
                PayloadAck(int(fields[2]), PayloadResult.SUCCESS)
            )))


def main(args=None) -> None:  # noqa: D103
    rclpy.init(args=args)
    node = PayloadMock()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
