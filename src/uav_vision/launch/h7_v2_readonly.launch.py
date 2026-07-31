"""The formal read-only OpenMV V2 chain: bridge and interface only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/dtask_openmv'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('data_timeout_sec', default_value='0.30'),
        DeclareLaunchArgument('allow_legacy_protocol', default_value='false'),
        DeclareLaunchArgument('closed_loop_enable', default_value='false'),
        DeclareLaunchArgument('closed_loop_min_fps', default_value='5.0'),
        DeclareLaunchArgument('calibration_profile', default_value=''),
        DeclareLaunchArgument('calibration_loaded', default_value='false'),
        Node(
            package='uav_vision', executable='h7_bridge_node', name='h7_bridge_node',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'data_timeout_sec': LaunchConfiguration('data_timeout_sec'),
                'allow_legacy_protocol': LaunchConfiguration('allow_legacy_protocol'),
            }], output='screen'),
        Node(package='uav_vision', executable='vision_interface_node',
             name='vision_interface_node', output='screen', parameters=[{
                 'closed_loop_enable': LaunchConfiguration('closed_loop_enable'),
                 'closed_loop_min_fps': LaunchConfiguration('closed_loop_min_fps'),
                 'calibration_profile': LaunchConfiguration('calibration_profile'),
                 'calibration_loaded': LaunchConfiguration('calibration_loaded'),
             }]),
    ])
