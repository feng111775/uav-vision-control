"""Unified first-task launch with explicit profile selection."""

from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _config(package: str, relative: str) -> str:
    return str(Path(get_package_share_directory(package)) / relative)


def _nodes(context):
    profile = LaunchConfiguration('profile').perform(context)
    if profile not in {'readonly_bench', 'no_prop_control', 'real_competition'}:
        raise RuntimeError(
            'profile must be readonly_bench, no_prop_control, or real_competition')
    control_cfg = _config(
        'uav_control', 'config/competition_emergency_final.yaml')
    vision_cfg = _config('uav_vision', 'config/openmv_v2_competition.yaml')
    acceptance_cfg = _config(
        'uav_vision', 'config/openmv_downward_v1_acceptance.yaml')
    if profile == 'real_competition':
        acceptance = yaml.safe_load(Path(acceptance_cfg).read_text())
        marker = acceptance['vision_interface_node']['ros__parameters']
        if (
            not marker['camera_orientation_verified']
            or not marker['coordinate_mapping_verified']
            or marker['camera_mount_profile'] == 'unverified'
        ):
            raise RuntimeError(
                'real_competition blocked: camera installation acceptance '
                'markers are not complete')
    car_cfg = _config(
        'uav_control',
        'config/car_udp_real.yaml' if profile == 'real_competition'
        else 'config/car_udp_localhost.yaml')
    servo_cfg = _config(
        'servo_control',
        'config/servo_real.yaml' if profile == 'real_competition'
        else 'config/servo_dry_run.yaml')
    enable_control = profile != 'readonly_bench'
    enable_auto_arm = False
    enable_release = profile == 'real_competition'
    vision_params = {
        'closed_loop_enable': profile == 'real_competition',
    }
    if profile != 'real_competition':
        vision_params.update({
            'camera_mount_profile': 'unverified',
            'camera_orientation_verified': False,
            'coordinate_mapping_verified': False,
        })
    return [
        Node(
            package='uav_control', executable='readiness_gate',
            name='readiness_gate_node', output='screen',
            parameters=[{
                'simulation_mode': False,
                'allow_sitl_heading_quality_bypass': False,
                'exit_on_ready': False,
            }]),
        Node(
            package='uav_vision', executable='h7_bridge_node',
            name='h7_bridge_node', output='screen',
            parameters=[vision_cfg]),
        Node(
            package='uav_vision', executable='vision_interface_node',
            name='vision_interface_node', output='screen',
            parameters=[vision_params, acceptance_cfg]),
        Node(
            package='uav_control', executable='car_start_gateway',
            name='car_start_gateway', output='screen',
            parameters=[car_cfg]),
        Node(
            package='uav_control', executable='mission_controller_node',
            name='mission_controller_node', output='screen',
            parameters=[control_cfg, {
                'enable_control': enable_control,
                'enable_auto_arm': enable_auto_arm,
                'enable_payload_release': enable_release,
                'vision_adapter_mode': 'formal_v2',
                'vision_health_required': True,
                'vision_health_topic': '/vision/health',
            }]),
        Node(
            package='servo_control', executable='servo_node',
            name='servo_node', output='screen',
            parameters=[servo_cfg, {
                'dry_run': profile != 'real_competition',
                'gpio_pin': 18,
            }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('profile', default_value='readonly_bench'),
        OpaqueFunction(function=_nodes),
    ])
