"""Standalone, operator-launched car UDP gateway."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_control',
            executable='car_start_gateway',
            name='car_start_gateway',
            parameters=[
                {'transport': 'udp'},
            ],
            output='screen'),
    ])
