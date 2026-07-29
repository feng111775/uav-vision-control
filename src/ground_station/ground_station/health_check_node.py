"""One-shot health check for the offline read-only ground station."""

import rclpy
from rclpy.node import Node


def health_status() -> dict:
    """Return the immutable-by-convention, read-only module status."""
    return {
        "status": "READY",
        "mode": "READ_ONLY",
        "offline": True,
        "control_commands": (),
    }


class HealthCheckNode(Node):
    """Report ground-station readiness without creating control interfaces."""

    def __init__(self) -> None:
        super().__init__("ground_station_health_check")
        status = health_status()
        self.get_logger().info(
            "ground_station status={status} mode={mode} offline={offline}".format(
                status=status["status"],
                mode=status["mode"],
                offline=str(status["offline"]).lower(),
            )
        )


def main(args=None) -> None:
    """Log one status line and exit without publishing control topics."""
    rclpy.init(args=args)
    node = HealthCheckNode()
    node.destroy_node()
    rclpy.shutdown()
