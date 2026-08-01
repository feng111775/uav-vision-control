"""The formal read-only OpenMV V2 chain: bridge, filter and predictor."""

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
    data_timeout_sec = LaunchConfiguration('data_timeout_sec')
    allow_legacy_protocol = LaunchConfiguration('allow_legacy_protocol')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/dtask_openmv'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('data_timeout_sec', default_value='0.30'),
        DeclareLaunchArgument('allow_legacy_protocol', default_value='false'),
        Node(
            package='uav_vision', executable='h7_bridge_node',
            name='h7_bridge_node',
            parameters=[config, {
                'port': port,
                'baudrate': baudrate,
                'data_timeout_sec': data_timeout_sec,
                'allow_legacy_protocol': allow_legacy_protocol,
            }],
            output='screen'),
        Node(package='uav_vision', executable='target_filter_node',
             name='target_filter_node', parameters=[config], output='screen'),
        Node(package='uav_vision', executable='target_predictor_node',
             name='target_predictor_node', parameters=[config],
             output='screen'),
        Node(package='uav_vision', executable='landing_error_node',
             name='landing_error_node', parameters=[config], output='screen'),
        Node(package='uav_vision', executable='vision_dashboard_node',
             name='vision_dashboard_node', parameters=[config],
             output='screen'),
    ])
