"""Launch only the formal controller and read-only mission dashboard."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PROFILES = {'sitl_drop', 'sitl_dynamic_land', 'first_flight_hover',
            'competition_drop', 'competition_dynamic_land'}


def _launch(context):
    profile = LaunchConfiguration('profile').perform(context)
    mode = LaunchConfiguration('mission_mode').perform(context)
    if profile not in PROFILES:
        raise RuntimeError('unsupported profile: ' + profile)
    if mode not in ('drop', 'dynamic_land', 'hover_test'):
        raise RuntimeError('invalid mission_mode')
    config = os.path.join(get_package_share_directory('uav_control'), 'config', profile + '.yaml')
    return [Node(package='uav_control',
                 executable='mission_controller_node',
                 name='mission_controller_node',
                 parameters=[config,
                             {'mission_mode': mode}],
                 output='screen'),
            Node(package='uav_control',
                 executable='mission_dashboard_node',
                 name='mission_dashboard_node',
                 output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('profile', default_value='first_flight_hover'),
                              DeclareLaunchArgument('mission_mode', default_value='hover_test'),
                              OpaqueFunction(function=_launch)])
