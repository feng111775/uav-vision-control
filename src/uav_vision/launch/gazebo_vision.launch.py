"""Launch the Gazebo-only image bridge and vision processing chain."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the vision launch description without arming PX4."""
    config = str(
        Path(get_package_share_directory('uav_vision'))
        / 'config' / 'gazebo_vision.yaml')
    target_model = str(
        Path(get_package_share_directory('uav_vision'))
        / 'models' / 'red_target_plate' / 'model.sdf')
    common = {'parameters': [config], 'output': 'screen'}
    gz_image_topic = LaunchConfiguration('gz_image_topic')
    gz_camera_info_topic = LaunchConfiguration('gz_camera_info_topic')
    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_image_topic',
            default_value=(
                '/world/default/model/x500_mono_cam_down_0/'
                'link/camera_link/sensor/imager/image'),
            description='Gazebo Transport image topic from the downward camera'),
        DeclareLaunchArgument(
            'gz_camera_info_topic',
            default_value=(
                '/world/default/model/x500_mono_cam_down_0/'
                'link/camera_link/sensor/imager/camera_info'),
            description='Gazebo Transport camera_info topic from the downward camera'),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_red_target_plate',
            arguments=[
                '-world', 'default',
                '-file', target_model,
                '-name', 'red_target_plate',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.01',
                '-allow_renaming', 'true',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='parameter_bridge',
            arguments=[
                [gz_image_topic, '@sensor_msgs/msg/Image[gz.msgs.Image'],
                [gz_camera_info_topic,
                 '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[
                (gz_image_topic, '/camera/down/image_raw'),
                (gz_camera_info_topic, '/camera/down/camera_info'),
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
            executable='target_predictor_node',
            name='target_predictor_node',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='landing_error_node',
            name='landing_error_node',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='visual_servo_node',
            name='visual_servo_node',
            **common,
        ),
        Node(
            package='uav_vision',
            executable='vision_dashboard_node',
            name='vision_dashboard_node',
            **common,
        ),
    ])
