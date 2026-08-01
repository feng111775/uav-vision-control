"""Single safe competition software entry point."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = Path(get_package_share_directory('uav_control'))
    vision_share = Path(get_package_share_directory('uav_vision'))
    servo_share = Path(get_package_share_directory('servo_control'))
    mission_config = str(control_share / 'config' /
                         'competition_emergency_final.yaml')
    udp_config = str(control_share / 'config' / 'car_udp_localhost.yaml')
    vision_config = str(vision_share / 'config' / 'red_target_hardware.yaml')
    servo_config = str(servo_share / 'config' / 'servo_dry_run.yaml')
    launch_vision = LaunchConfiguration('launch_vision')
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_vision', default_value='true',
            description='Start the real camera/H7 vision chain'),
        DeclareLaunchArgument(
            'udp_port', default_value='19001',
            description='UDP listener port for the ESP32 START gateway'),
        Node(
            package='uav_control', executable='car_start_gateway',
            name='car_start_gateway', output='screen',
            parameters=[udp_config, {
                'simulation_mode': False,
                'communication_only': False,
                'udp_port': LaunchConfiguration('udp_port'),
            }]),
        Node(
            package='uav_control', executable='mission_controller_node',
            name='mission_controller_node', output='screen',
            parameters=[mission_config]),
        Node(
            package='servo_control', executable='servo_node',
            name='servo_node', output='screen',
            parameters=[servo_config, {
                'dry_run': True,
                'gpio_pin': 18,
            }]),
        Node(
            package='pi_camera_vision', executable='pi_camera_vision_node',
            name='pi_camera_vision_node', output='screen',
            condition=IfCondition(launch_vision),
            parameters=[vision_config]),
        Node(
            package='uav_vision', executable='h7_bridge_node',
            name='h7_bridge_node', output='screen',
            condition=IfCondition(launch_vision),
            parameters=[vision_config]),
        Node(
            package='uav_vision', executable='camera_selector_node',
            name='camera_selector_node', output='screen',
            condition=IfCondition(launch_vision),
            parameters=[vision_config]),
        Node(
            package='uav_vision', executable='target_filter_node',
            name='target_filter_node', output='screen',
            condition=IfCondition(launch_vision),
            parameters=[vision_config]),
        Node(
            package='uav_vision', executable='target_predictor_node',
            name='target_predictor_node', output='screen',
            condition=IfCondition(launch_vision),
            parameters=[vision_config, {
                'filtered_detection_topic': '/vision/selected_filtered_detection',
                'tracked_topic': '/vision/target/tracked',
            }]),
        Node(
            package='uav_vision', executable='landing_error_node',
            name='landing_error_node', output='screen',
            condition=IfCondition(launch_vision),
            parameters=[vision_config]),
    ])
