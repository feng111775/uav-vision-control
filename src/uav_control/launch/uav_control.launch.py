"""Launch the formal UAV vision Offboard controller."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start only the formal vision Offboard controller."""
    config = str(
        Path(get_package_share_directory('uav_control'))
        / 'config' / 'control.yaml')

    return LaunchDescription([
        Node(
            package='uav_control',
            executable='vision_offboard_controller',
            name='vision_offboard_controller',
            parameters=[config],
            output='screen',
        ),
    ])
