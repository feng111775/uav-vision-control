"""Launch one camera detector source and the safe vision backend only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start Pi camera, filter, and visual servo without PX4 control."""
    config = str(Path(get_package_share_directory('pi_camera_vision'))
                 / 'config' / 'pi_camera_vision.yaml')
    common = {'parameters': [config], 'output': 'screen'}
    return LaunchDescription([
        Node(
            package='pi_camera_vision',
            executable='pi_camera_vision_node',
            name='pi_camera_vision_node',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='target_filter_node',
            name='target_filter_node',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='visual_servo_node',
            name='visual_servo_node',
            **common,
        ),
    ])
