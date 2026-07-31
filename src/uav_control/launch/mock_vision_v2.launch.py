"""Opt-in V2 mock publisher; it never starts mission control or PX4."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='healthy_no_target'),
        Node(
            package='uav_control', executable='mock_vision_v2_node',
            name='mock_vision_v2_node', output='screen', parameters=[{
                'simulation_mode': True,
                'scenario': LaunchConfiguration('scenario'),
            }]),
    ])
