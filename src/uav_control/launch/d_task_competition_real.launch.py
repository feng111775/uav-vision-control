"""Real competition entry point; defaults to disarmed, dry-run safe mode."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control = Path(get_package_share_directory('uav_control'))
    vision = Path(get_package_share_directory('uav_vision'))
    servo = Path(get_package_share_directory('servo_control'))
    mission_executable = 'mission_' + 'controller_node'
    return LaunchDescription([
        DeclareLaunchArgument('flight_authorized', default_value='false'),
        DeclareLaunchArgument('payload_authorized', default_value='false'),
        DeclareLaunchArgument('udp_bind_address', default_value='0.0.0.0'),
        DeclareLaunchArgument('udp_port', default_value='19001'),
        DeclareLaunchArgument('esp32_peer_ip', default_value=''),
        DeclareLaunchArgument('launch_vision', default_value='true'),
        Node(package='uav_control', executable='readiness_gate',
             name='readiness_gate',
             parameters=[str(control / 'config' / 'competition_real.yaml'), {
                 'simulation_mode': False,
                 'flight_authorized': LaunchConfiguration('flight_authorized'),
                 'payload_authorized': LaunchConfiguration(
                     'payload_authorized')}]),
        Node(package='uav_control', executable='car_start_gateway', name='car_start_gateway',
             parameters=[str(control / 'config' / 'car_udp_real.yaml'), {
                 'udp_host': LaunchConfiguration('udp_bind_address'),
                 'udp_port': LaunchConfiguration('udp_port'),
                 'udp_peer_host': LaunchConfiguration('esp32_peer_ip'),
                 'udp_require_peer': True}]),
        Node(package='uav_control', executable=mission_executable,
             name='mission_controller_node',
             parameters=[str(control / 'config' / 'competition_real.yaml'), {
                 'flight_authorized': LaunchConfiguration('flight_authorized'),
                 'payload_authorized': LaunchConfiguration('payload_authorized'),
                 'enable_control': LaunchConfiguration('flight_authorized'),
                 'enable_auto_arm': LaunchConfiguration('flight_authorized'),
                 'enable_payload_release': LaunchConfiguration('payload_authorized'),
                 'task_id': ''}]),
        Node(package='servo_control', executable='servo_node', name='servo_node',
             parameters=[str(servo / 'config' / 'servo_real.yaml'), {
                 'payload_authorized': LaunchConfiguration('payload_authorized'),
                 'servo_dry_run': ParameterValue(
                     PythonExpression(["'", LaunchConfiguration(
                         'payload_authorized'), "' != 'true'"]),
                     value_type=bool)}]),
        Node(package='uav_vision', executable='h7_bridge_node', name='h7_bridge_node',
             parameters=[str(vision / 'config' / 'd_task_vision.yaml')]),
        Node(package='uav_vision', executable='target_filter_node',
             name='target_filter_node',
             parameters=[str(vision / 'config' / 'd_task_vision.yaml')]),
        Node(package='uav_vision', executable='target_predictor_node',
             name='target_predictor_node',
             parameters=[str(vision / 'config' / 'd_task_vision.yaml')]),
        Node(package='uav_vision', executable='landing_error_node', name='landing_error_node',
             parameters=[str(vision / 'config' / 'd_task_vision.yaml')]),
        Node(package='uav_vision', executable='visual_servo_node', name='visual_servo_node',
             parameters=[str(vision / 'config' / 'd_task_vision.yaml')]),
    ])
