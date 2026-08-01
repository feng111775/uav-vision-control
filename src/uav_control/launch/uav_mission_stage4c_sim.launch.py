"""Launch the five stage 4C nodes in software-only safe mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the stage 4C simulation launch description."""
    default_config = os.path.join(
        get_package_share_directory('uav_control'),
        'config', 'mission_stage4c.yaml')
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
            description='Stage 4C parameter YAML'),
        *[
            Node(
                package='uav_control',
                executable=name,
                name=name,
                output='screen',
                parameters=[config, {
                    'simulation_mode': True,
                    'dry_run': True,
                    'enable_control': False,
                    'enable_auto_arm': False,
                }])
            for name in nodes
        ],
    ])
