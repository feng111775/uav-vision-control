"""Fail-closed, propeller-free real-Pixhawk monitoring entry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    """Start five task nodes with all control and physical output disabled."""
    config = os.path.join(
        get_package_share_directory('uav_control'), 'config',
        'mission_stage4c_hardware_bench.yaml')
    nodes = (
        'car_start_gateway',
        'car_marker_vision',
        'payload_release',
        'mission_manager',
    )
    return LaunchDescription([
        LogInfo(msg=(
            'HARDWARE BENCH MONITOR ONLY: control=false auto_arm=false '
            'physical_release=false; no PX4 input publisher is launched')),
        *[
            Node(
                package='uav_control', executable=name, name=name,
                output='screen', parameters=[config])
            for name in nodes
        ],
    ])
