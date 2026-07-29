"""One-shot ROS distribution check for joint simulation."""

import os

import rclpy
from rclpy.node import Node


def system_status(environ=None) -> dict:
    """Read the ROS distribution and return an explicit readiness state."""
    environment = os.environ if environ is None else environ
    distro = environment.get("ROS_DISTRO")
    return {
        "status": "READY" if distro == "jazzy" else "MISMATCH",
        "ros_distro": distro if distro else "unknown",
    }


class SystemCheckNode(Node):
    """Report whether the active ROS distribution is Jazzy."""

    def __init__(self) -> None:
        super().__init__("d_system_sim_check")
        status = system_status()
        self.get_logger().info(
            f"d_system_sim status={status['status']} "
            f"ros_distro={status['ros_distro']}"
        )


def main(args=None) -> None:
    """Log one environment check and exit without starting Gazebo or PX4."""
    rclpy.init(args=args)
    node = SystemCheckNode()
    node.destroy_node()
    rclpy.shutdown()
