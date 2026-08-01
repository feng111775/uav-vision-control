"""Run a D-task stage from the full safe initialization path in SITL."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav_control.stage_catalog import validate_stage_target


def _launch(context):
    mode = LaunchConfiguration('mission_mode').perform(context)
    if mode not in ('drop', 'dynamic_land'):
        raise RuntimeError('D-task mode must be drop or dynamic_land')
    stage = validate_stage_target(
        LaunchConfiguration('target_stage').perform(context))
    control = os.path.join(
        get_package_share_directory('uav_control'), 'config',
        'sitl_' + mode + '.yaml')
    observer = Node(
        package='uav_control', executable='stage_observer',
        name='stage_observer', output='screen',
        parameters=[{
            'target_stage': stage,
            'timeout_seconds': LaunchConfiguration('timeout_seconds'),
        }])
    return [
        Node(
            package='uav_control', executable='mission_controller_node',
            name='mission_controller_node', output='screen',
            parameters=[
                control,
                {
                    'simulation_mode': True,
                    'allow_sitl_heading_quality_bypass': True,
                },
            ]),
        Node(
            package='uav_control', executable='d_task_mock_node',
            name='d_task_mock_node', output='screen',
            parameters=[{'simulation_mode': True}]),
        observer,
        RegisterEventHandler(OnProcessExit(
            target_action=observer,
            on_exit=[EmitEvent(event=Shutdown(
                reason='requested D-task stage test finished'))])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission_mode', default_value='drop'),
        DeclareLaunchArgument('target_stage', default_value='SEARCH_CAR'),
        DeclareLaunchArgument('timeout_seconds', default_value='90.0'),
        OpaqueFunction(function=_launch),
    ])
