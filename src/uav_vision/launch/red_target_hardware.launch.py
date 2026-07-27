"""Launch the first-flight dual-camera chain with safe hardware defaults."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start both hardware cameras and a manually enabled controller."""
    config = str(
        Path(get_package_share_directory('uav_vision'))
        / 'config' / 'red_target_hardware.yaml')
    enable_offboard = ParameterValue(
        LaunchConfiguration('enable_offboard'), value_type=bool)
    common = {'parameters': [config], 'output': 'screen'}

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_offboard',
            default_value='false',
            description='Explicitly enable Offboard output after monitoring'),
        Node(
            package='pi_camera_vision',
            executable='pi_camera_vision_node',
            name='pi_camera_vision_node',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='h7_bridge_node',
            name='h7_bridge_node',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='camera_selector_node',
            name='camera_selector_node',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='target_filter_node',
            name='target_filter_node',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='visual_servo_node',
            name='visual_servo_node',
            **common,
        ),
        Node(
            package='uav_control',
            executable='vision_offboard_controller',
            name='vision_offboard_controller',
            parameters=[
                config,
                {
                    'simulation_mode': False,
                    'enable_offboard': enable_offboard,
                    'enable_auto_arm': False,
                },
            ],
            output='screen',
        ),
    ])
