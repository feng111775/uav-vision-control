"""Launch the safe dual-camera Gazebo vision-to-PX4 ROS chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start two detectors, one selector, and the single formal controller."""
    config = str(
        Path(get_package_share_directory('uav_vision'))
        / 'config' / 'dual_camera_simulation.yaml')
    common = {'parameters': [config], 'output': 'screen'}
    selector_mode = LaunchConfiguration('selector_mode')
    enable_offboard = ParameterValue(
        LaunchConfiguration('enable_offboard'), value_type=bool)
    enable_auto_arm = ParameterValue(
        LaunchConfiguration('enable_auto_arm'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument(
            'selector_mode',
            default_value='auto',
            description='Detection selection mode: front, down, or auto'),
        DeclareLaunchArgument(
            'enable_offboard',
            default_value='false',
            description='Enable PX4 Offboard output in SITL only'),
        DeclareLaunchArgument(
            'enable_auto_arm',
            default_value='false',
            description='Allow automatic arming in SITL only'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='dual_camera_bridge',
            arguments=[
                '/camera/front/image'
                '@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/front/camera_info'
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/camera/down/image'
                '@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/down/camera_info'
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[
                ('/camera/front/image', '/camera/front/image_raw'),
                ('/camera/down/image', '/camera/down/image_raw'),
            ],
            **common,
        ),
        Node(
            package='uav_vision',
            executable='gazebo_red_target_detector_node',
            name='front_red_target_detector',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='gazebo_red_target_detector_node',
            name='down_red_target_detector',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='camera_selector_node',
            name='camera_selector_node',
            parameters=[config, {'mode': selector_mode}],
            output='screen',
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
                    'simulation_mode': True,
                    'enable_offboard': enable_offboard,
                    'enable_auto_arm': enable_auto_arm,
                },
            ],
            output='screen',
        ),
    ])
