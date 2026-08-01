"""Single competition entry point with fail-closed defaults."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control = Path(get_package_share_directory('uav_control'))
    vision = Path(get_package_share_directory('uav_vision'))
    servo = Path(get_package_share_directory('servo_control'))
    mission = str(control / 'config' / 'competition_drop.yaml')
    vision_config = str(vision / 'config' / 'red_target_hardware.yaml')
    enable_control = ParameterValue(
        LaunchConfiguration('enable_control'), value_type=bool)
    enable_auto_arm = ParameterValue(
        LaunchConfiguration('enable_auto_arm'), value_type=bool)
    enable_payload = ParameterValue(
        LaunchConfiguration('enable_payload_release'), value_type=bool)
    enable_auto_disarm = ParameterValue(
        LaunchConfiguration('enable_auto_disarm'), value_type=bool)
    servo_dry_run = ParameterValue(
        LaunchConfiguration('servo_dry_run'), value_type=bool)
    launch_vision = LaunchConfiguration('launch_vision')
    return LaunchDescription([
        DeclareLaunchArgument('enable_control', default_value='false'),
        DeclareLaunchArgument('enable_auto_arm', default_value='false'),
        DeclareLaunchArgument('enable_auto_disarm', default_value='false'),
        DeclareLaunchArgument('enable_payload_release', default_value='false'),
        DeclareLaunchArgument('servo_dry_run', default_value='true'),
        DeclareLaunchArgument('launch_vision', default_value='false'),
        Node(package='uav_control', executable='readiness_gate',
             name='readiness_gate', parameters=[mission]),
        Node(package='uav_control', executable='car_start_gateway',
             name='car_start_gateway', parameters=[{
                 'simulation_mode': False,
                 'transport': 'udp',
                 'real_transport_enabled': True,
                 'udp_host': '0.0.0.0',
                 'udp_port': 19001,
                 'udp_require_peer': False,
             }]),
        Node(package='uav_control', executable='mission_controller_node',
             name='mission_controller_node',
             parameters=[mission, {
                 'enable_control': enable_control,
                 'enable_auto_arm': enable_auto_arm,
                 'enable_auto_disarm': enable_auto_disarm,
                 'enable_payload_release': enable_payload,
             }]),
        Node(package='servo_control', executable='servo_node',
             name='servo_node',
             parameters=[str(servo / 'config' / 'servo.yaml'),
                         {'servo_dry_run': servo_dry_run}]),
        Node(package='pi_camera_vision', executable='pi_camera_vision_node',
             name='pi_camera_vision_node',
             condition=IfCondition(launch_vision),
             parameters=[vision_config]),
        Node(package='uav_vision', executable='h7_bridge_node',
             name='h7_bridge_node', condition=IfCondition(launch_vision),
             parameters=[vision_config]),
        Node(package='uav_vision', executable='camera_selector_node',
             name='camera_selector_node', condition=IfCondition(launch_vision),
             parameters=[vision_config]),
        Node(package='uav_vision', executable='target_filter_node',
             name='target_filter_node', condition=IfCondition(launch_vision),
             parameters=[vision_config]),
        Node(package='uav_vision', executable='target_predictor_node',
             name='target_predictor_node', condition=IfCondition(launch_vision),
             parameters=[vision_config]),
        Node(package='uav_vision', executable='landing_error_node',
             name='landing_error_node', condition=IfCondition(launch_vision),
             parameters=[vision_config]),
        Node(package='uav_vision', executable='visual_servo_node',
             name='visual_servo_node', condition=IfCondition(launch_vision),
             parameters=[vision_config]),
    ])
