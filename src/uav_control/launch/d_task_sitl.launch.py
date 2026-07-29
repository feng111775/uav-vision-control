"""D-task ROS scenario launch; PX4 and Agent must be started separately."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    mode = LaunchConfiguration('mission_mode').perform(context)
    if mode not in ('drop', 'dynamic_land'):
        raise RuntimeError('SITL mode must be drop or dynamic_land')
    control = os.path.join(get_package_share_directory(
        'uav_control'), 'config', 'sitl_' + mode + '.yaml')
    vision = os.path.join(get_package_share_directory(
        'uav_vision'), 'config', 'd_task_vision.yaml')
    fault_mode = LaunchConfiguration('fault_mode').perform(context)
    mission_timeout = float(
        LaunchConfiguration('mission_timeout_seconds').perform(context))
    control_parameters = [control]
    if mission_timeout > 0.0:
        control_parameters.append(
            {'mission_timeout_seconds': mission_timeout})
    executables = [
        'target_filter_node',
        'target_predictor_node',
        'landing_error_node',
        'vision_dashboard_node']
    nodes = [Node(package='uav_control',
                  executable='mission_controller_node',
                  name='mission_controller_node',
                  parameters=control_parameters,
                  output='screen'),
             Node(package='uav_control',
             executable='mission_dashboard_node',
             name='mission_dashboard_node',
             output='screen'),
             Node(package='uav_vision',
             executable='d_task_sitl_scenario_node',
             name='d_task_sitl_scenario_node',
             parameters=[{'mission_mode': mode, 'fault_mode': fault_mode}],
             output='screen')]
    nodes.extend(
        Node(
            package='uav_vision',
            executable=e,
            name=e,
            parameters=[vision],
            output='screen') for e in executables)
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mission_mode', default_value='drop'),
        DeclareLaunchArgument('fault_mode', default_value='none'),
        DeclareLaunchArgument(
            'mission_timeout_seconds', default_value='-1.0'),
        OpaqueFunction(function=_launch)])
