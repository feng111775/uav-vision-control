"""Single competition entry point with fail-closed defaults."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    control = Path(get_package_share_directory('uav_control'))
    vision = Path(get_package_share_directory('uav_vision'))
    camera = Path(get_package_share_directory('pi_camera_vision'))
    servo = Path(get_package_share_directory('servo_control'))
    mission = str(control / 'config' / 'competition_drop.yaml')
    vision_config = str(vision / 'config' / 'd_task_vision.yaml')
    camera_config = str(camera / 'config' / 'pi_camera_vision.yaml')
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
    simulation_mode = ParameterValue(
        LaunchConfiguration('simulation_mode'), value_type=bool)
    allow_sitl_heading_quality_bypass = ParameterValue(
        LaunchConfiguration('allow_sitl_heading_quality_bypass'),
        value_type=bool)
    camera_source_type = LaunchConfiguration('camera_source_type')
    camera_source = LaunchConfiguration('camera_source')
    camera_device = ParameterValue(
        LaunchConfiguration('camera_device'), value_type=int)
    h7_port = LaunchConfiguration('port')
    h7_baudrate = ParameterValue(
        LaunchConfiguration('baudrate'), value_type=int)
    h7_data_timeout = LaunchConfiguration('data_timeout_sec')
    h7_allow_legacy = LaunchConfiguration('allow_legacy_protocol')
    launch_vision = LaunchConfiguration('launch_vision')
    launch_h7_bridge = LaunchConfiguration('launch_h7_bridge')
    return LaunchDescription([
        DeclareLaunchArgument('enable_control', default_value='false'),
        DeclareLaunchArgument('enable_auto_arm', default_value='false'),
        DeclareLaunchArgument('enable_auto_disarm', default_value='false'),
        DeclareLaunchArgument('enable_payload_release', default_value='false'),
        DeclareLaunchArgument('servo_dry_run', default_value='true'),
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument(
            'allow_sitl_heading_quality_bypass', default_value='false'),
        DeclareLaunchArgument('launch_vision', default_value='false'),
        DeclareLaunchArgument('launch_h7_bridge', default_value='false'),
        DeclareLaunchArgument('camera_source_type', default_value='usb'),
        DeclareLaunchArgument('camera_source', default_value=''),
        DeclareLaunchArgument('camera_device', default_value='0'),
        DeclareLaunchArgument('port', default_value='/dev/dtask_openmv'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('data_timeout_sec', default_value='0.30'),
        DeclareLaunchArgument('allow_legacy_protocol', default_value='false'),
        Node(package='uav_control', executable='readiness_gate',
             name='readiness_gate', parameters=[mission, {
                 'simulation_mode': simulation_mode,
                 'allow_sitl_heading_quality_bypass':
                     allow_sitl_heading_quality_bypass,
             }]),
        Node(package='uav_control', executable='car_start_gateway',
             name='car_start_gateway', parameters=[{
                 'simulation_mode': simulation_mode,
                 'transport': 'udp',
                 'real_transport_enabled': True,
                 'udp_host': '0.0.0.0',
                 'udp_port': 19001,
                 'udp_require_peer': False,
             }]),
        Node(package='uav_control', executable='mission_controller_node',
             name='mission_controller_node',
             parameters=[mission, {
                 'simulation_mode': simulation_mode,
                 'allow_sitl_heading_quality_bypass':
                     allow_sitl_heading_quality_bypass,
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
             condition=IfCondition(PythonExpression([
                 "'", launch_vision, "' == 'true' and '", launch_h7_bridge,
                 "' == 'false'"])),
             parameters=[camera_config, {
                 'source_type': camera_source_type,
                 'source': camera_source,
                 'device': camera_device,
             }]),
        Node(package='uav_vision', executable='h7_bridge_node',
             name='h7_bridge_node',
             condition=IfCondition(PythonExpression([
                 "'", launch_vision, "' == 'true' and '", launch_h7_bridge,
                 "' == 'true'"])),
             parameters=[vision_config, {
                 'port': h7_port,
                 'baudrate': h7_baudrate,
                 'data_timeout_sec': h7_data_timeout,
                 'allow_legacy_protocol': h7_allow_legacy,
             }]),
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
