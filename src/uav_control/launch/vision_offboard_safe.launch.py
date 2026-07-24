"""Launch the controller with hardware-safe defaults from YAML."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start the controller disabled for static ROS interface checks."""
    config = str(Path(get_package_share_directory('uav_control'))
                 / 'config' / 'vision_offboard.yaml')
    return LaunchDescription([
        Node(
            package='uav_control',
            executable='vision_offboard_controller',
            name='vision_offboard_controller',
            parameters=[config],
            output='screen',
        ),
    ])
