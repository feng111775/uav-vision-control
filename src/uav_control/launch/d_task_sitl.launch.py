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
    executables = [
        'target_filter_node',
        'target_predictor_node',
        'landing_error_node',
        'vision_dashboard_node']
    nodes = [Node(package='uav_control',
                  executable='mission_controller_node',
                  name='mission_controller_node',
                  parameters=[control],
                  output='screen'),
             Node(package='uav_control',
             executable='mission_dashboard_node',
             name='mission_dashboard_node',
             output='screen'),
             Node(package='uav_vision',
                  executable='d_task_sitl_scenario_node',
                  name='d_task_sitl_scenario_node',
                  parameters=[{'mission_mode': mode}],
                  # Temporary compatibility only. Remove after uav_vision
                  # changes its stale PX4 v1.16 local-position topic name.
                  remappings=[('/fmu/out/vehicle_local_position_v1',
                               '/fmu/out/vehicle_local_position')],
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
    return LaunchDescription([DeclareLaunchArgument(
        'mission_mode', default_value='drop'), OpaqueFunction(function=_launch)])
