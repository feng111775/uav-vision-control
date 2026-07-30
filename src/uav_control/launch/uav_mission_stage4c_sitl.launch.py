"""Launch the five task nodes against real PX4 SITL DDS output."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the stage 4C SITL-only launch description."""
    default_config = os.path.join(
        get_package_share_directory('uav_control'),
        'config', 'mission_stage4c_sitl.yaml')
    config = LaunchConfiguration('config_file')
    mission_timeout = ParameterValue(
        LaunchConfiguration('mission_timeout_s'), value_type=float)
    follow_timeout = ParameterValue(
        LaunchConfiguration('follow_timeout_s'), value_type=float)
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
        DeclareLaunchArgument(
            'mission_timeout_s', default_value='90.0',
            description='Global acceptance timeout; production default 90 s'),
        DeclareLaunchArgument(
            'follow_timeout_s', default_value='15.0',
            description='FOLLOW timeout; acceptance override only'),
        *[
            Node(
                package='uav_control',
                executable=name,
                name=name,
                output='screen',
                parameters=(
                    [config, {
                        'mission_timeout_s': mission_timeout,
                        'follow_timeout_s': follow_timeout,
                    }] if name == 'mission_manager' else [config]))
            for name in nodes
        ],
    ])
