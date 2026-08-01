"""Realtime OpenMV bridge and array-based downstream vision chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vision = Path(get_package_share_directory('uav_vision'))
    config = str(vision / 'config' / 'd_task_vision.yaml')
    port = LaunchConfiguration('port')
    baudrate = ParameterValue(LaunchConfiguration('baudrate'),
                              value_type=int)
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/dtask_openmv'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        Node(
            package='uav_vision', executable='h7_bridge_node',
            name='h7_bridge_node',
            parameters=[config, {
                'port': port,
                'baudrate': baudrate,
            }],
            output='screen'),
        Node(package='uav_vision', executable='target_filter_node',
             name='target_filter_node', parameters=[config],
             output='screen'),
        Node(package='uav_vision', executable='target_predictor_node',
             name='target_predictor_node', parameters=[config],
             output='screen'),
        Node(package='uav_vision', executable='landing_error_node',
             name='landing_error_node', parameters=[config],
             output='screen'),
    ])
