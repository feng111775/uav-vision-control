"""Launch the Gazebo-only image bridge and vision processing chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Build the vision launch description without arming PX4."""
    config = str(
        Path(get_package_share_directory('uav_vision'))
        / 'config' / 'gazebo_vision.yaml')
    common = {'parameters': [config], 'output': 'screen'}
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            arguments=[
                '/camera/down/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/down/camera_info'
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[
                ('/camera/down/image', '/camera/down/image_raw'),
            ],
            **common,
        ),
        Node(
            package='uav_vision',
            executable='gazebo_red_target_detector_node',
            name='gazebo_red_target_detector_node',
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
    ])
