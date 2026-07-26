"""Hardware-facing launch using the same mission and controller code."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('uav_vision'))
    config = str(share / 'config' / 'qr_shelf_hardware.yaml')
    target = LaunchConfiguration('target_qr_id')
    front = LaunchConfiguration('front_image_topic')
    return LaunchDescription([
        DeclareLaunchArgument('target_qr_id', default_value='7'),
        DeclareLaunchArgument(
            'front_image_topic', default_value='/camera/front/image_raw'),
        DeclareLaunchArgument(
            'down_image_topic', default_value='/camera/down/image_raw'),
        Node(
            package='uav_vision', executable='qr_detector_node',
            parameters=[config, {
                'target_qr_id': ParameterValue(target, value_type=int),
                'image_topic': front,
                'layout_path': str(
                    share / 'config' / 'qr_shelf_layout.yaml')}]),
        Node(
            package='uav_vision',
            executable='gazebo_red_target_detector_node',
            name='down_red_target_detector',
            parameters=[{
                'use_sim_time': False,
                'image_topic': LaunchConfiguration('down_image_topic'),
                'detection_topic': '/vision/down/detection'}]),
        Node(
            package='uav_vision', executable='camera_selector_node',
            parameters=[str(share / 'config' /
                            'dual_camera_simulation.yaml'),
                        {'use_sim_time': False}]),
        Node(
            package='uav_vision', executable='target_filter_node',
            parameters=[str(share / 'config' /
                            'dual_camera_simulation.yaml'),
                        {'use_sim_time': False}]),
        Node(
            package='uav_vision', executable='visual_servo_node',
            parameters=[str(share / 'config' /
                            'dual_camera_simulation.yaml'),
                        {'use_sim_time': False}]),
        Node(
            package='uav_control',
            executable='vision_offboard_controller',
            parameters=[config, {
                'target_qr_id': ParameterValue(target, value_type=int),
                # These remain false and are intentionally not launch args.
                'simulation_mode': False,
                'enable_offboard': False,
                'enable_auto_arm': False}]),
    ])
