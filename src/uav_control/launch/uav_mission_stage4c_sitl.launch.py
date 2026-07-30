"""Launch the five task nodes against real PX4 SITL DDS output."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the stage 4C SITL-only launch description."""
    default_config = os.path.join(
        get_package_share_directory('uav_control'),
        'config', 'mission_stage4c_sitl.yaml')
    config = LaunchConfiguration('config_file')
    nodes = [
        'car_start_gateway',
        'car_marker_vision',
        'mission_offboard_controller',
        'payload_release',
        'mission_manager',
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='PX4 SITL-only stage 4C parameters'),
        *[
            Node(
                package='uav_control',
                executable=name,
                name=name,
                output='screen',
                parameters=[config])
            for name in nodes
        ],
    ])
