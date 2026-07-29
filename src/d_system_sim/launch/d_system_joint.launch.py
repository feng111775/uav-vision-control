"""Stage-1 joint launch framework; no simulator, model, or PX4 is started."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch only the one-shot environment check."""
    return LaunchDescription([
        Node(
            package="d_system_sim",
            executable="system_check_node",
            output="screen",
        ),
    ])
