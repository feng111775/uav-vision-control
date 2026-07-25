"""Launch one camera detector source and the safe vision backend only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def camera_node(context, config):
    """Create the detector unless the default video path is still empty."""
    source_type = LaunchConfiguration('source_type').perform(context)
    source = LaunchConfiguration('source').perform(context)
    device = LaunchConfiguration('device').perform(context)
    if source_type == 'video' and not source.strip():
        return [LogInfo(msg=(
            'pi_camera_vision: source_type=video but source is empty; '
            'camera node is disabled until a video path is supplied'))]
    return [Node(
        package='pi_camera_vision',
        executable='pi_camera_vision_node',
        name='pi_camera_vision_node',
        parameters=[
            config,
            {
                'source_type': source_type,
                'source': source,
                'device': int(device),
            },
        ],
        output='screen',
    )]


def generate_launch_description():
    """Start Pi camera, filter, and visual servo without PX4 control."""
    config = str(Path(get_package_share_directory('pi_camera_vision'))
                 / 'config' / 'pi_camera_vision.yaml')
    common = {'parameters': [config], 'output': 'screen'}
    return LaunchDescription([
        DeclareLaunchArgument(
            'source_type',
            default_value='video',
            description='Camera backend: video, usb, or picamera2'),
        DeclareLaunchArgument(
            'source',
            default_value='',
            description='Video path (required when source_type=video)'),
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='USB camera index'),
        OpaqueFunction(function=camera_node, args=[config]),
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
