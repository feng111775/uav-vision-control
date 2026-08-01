"""Run one guarded hover scenario against an already running PX4 SITL."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SCENARIO_DEFAULTS = {
    'low_short': (0.5, 5.0),
    'nominal': (1.0, 10.0),
    'high_long': (1.5, 15.0),
}


def _launch(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    if scenario not in set(SCENARIO_DEFAULTS) | {'matrix'}:
        raise RuntimeError('unsupported SITL hover scenario: ' + scenario)
    default_height, default_duration = SCENARIO_DEFAULTS.get(
        scenario, SCENARIO_DEFAULTS['nominal'])
    height_text = LaunchConfiguration('target_height_m').perform(context)
    duration_text = LaunchConfiguration(
        'commanded_hover_duration_s').perform(context)
    target_height = (
        default_height if not height_text else float(height_text))
    hover_duration = (
        default_duration if not duration_text else float(duration_text))
    if not 0.5 <= target_height <= 2.0:
        raise RuntimeError('target_height_m must be in [0.5, 2.0]')
    if hover_duration <= 0.0:
        raise RuntimeError('commanded_hover_duration_s must be positive')
    profile = scenario if scenario in SCENARIO_DEFAULTS else 'nominal'
    config = os.path.join(
        get_package_share_directory('uav_control'), 'config',
        'sitl_hover_' + profile + '.yaml')
    control_overrides = {
        'target_altitude': target_height,
        'hover_test_seconds': hover_duration,
    }
    recorder = Node(
        package='uav_control', executable='sitl_result_recorder',
        name='sitl_result_recorder', output='screen',
        parameters=[{
            'scenario': scenario,
            'result_file': LaunchConfiguration('result_file'),
            'timeout_seconds': LaunchConfiguration('timeout_seconds'),
            'target_height_m': target_height,
            'commanded_hover_duration_s': hover_duration,
            'altitude_tolerance_m': 0.1,
        }])
    return [
        Node(
            package='uav_control', executable='mission_controller_node',
            name='mission_controller_node', output='screen',
            parameters=[config, control_overrides]),
        recorder,
        RegisterEventHandler(OnProcessExit(
            target_action=recorder,
            on_exit=[EmitEvent(event=Shutdown(
                reason='SITL recorder finished'))])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='nominal'),
        DeclareLaunchArgument('result_file', default_value=''),
        DeclareLaunchArgument('timeout_seconds', default_value='90.0'),
        DeclareLaunchArgument('target_height_m', default_value=''),
        DeclareLaunchArgument(
            'commanded_hover_duration_s', default_value=''),
        OpaqueFunction(function=_launch),
    ])
