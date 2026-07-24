"""Launch only the Pi camera vision publisher (never h7_bridge_node)."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Create the standalone vision launch description."""
    config = str(Path(get_package_share_directory('pi_camera_vision'))
                 / 'config' / 'pi_camera_vision.yaml')
    return LaunchDescription([
        Node(
            package='pi_camera_vision',
            executable='pi_camera_vision_node',
            name='pi_camera_vision_node',
            parameters=[config],
            output='screen',
        ),
    ])
