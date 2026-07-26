"""Launch offline/observe/SITL QR shelf modes with safe defaults."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('uav_vision'))
    mode = LaunchConfiguration('mode')
    arguments = [
        DeclareLaunchArgument('mode', default_value='observe',
                              choices=['offline', 'observe', 'sitl']),
        DeclareLaunchArgument('image_topic',
                              default_value='/camera/front/image_raw'),
        DeclareLaunchArgument('target_qr_id', default_value='7'),
        DeclareLaunchArgument('inventory_mode', default_value='target'),
        DeclareLaunchArgument('detector_backend', default_value='opencv'),
        DeclareLaunchArgument(
            'model_path',
            default_value=str(share / 'models' / 'qr_hog_svm.xml')),
        DeclareLaunchArgument('confidence_threshold', default_value='0.35'),
        DeclareLaunchArgument('confirm_frames', default_value='3'),
        DeclareLaunchArgument('qr_timeout', default_value='30.0'),
        DeclareLaunchArgument('laser_alignment_threshold',
                              default_value='12.0'),
        DeclareLaunchArgument('search_yaw_rate', default_value='0.2'),
        DeclareLaunchArgument('enable_offboard', default_value='false'),
        DeclareLaunchArgument('enable_auto_arm', default_value='false'),
        DeclareLaunchArgument('simulation_mode', default_value='true'),
        DeclareLaunchArgument('visualization', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true')]
    detector = Node(
        package='uav_vision', executable='qr_detector_node',
        name='qr_detector_node', output='screen',
        parameters=[str(share / 'config' / 'qr_shelf.yaml'), {
            'image_topic': LaunchConfiguration('image_topic'),
            'target_qr_id': ParameterValue(
                LaunchConfiguration('target_qr_id'), value_type=int),
            'search_yaw_rate': ParameterValue(
                LaunchConfiguration('search_yaw_rate'), value_type=float),
            'inventory_mode': LaunchConfiguration('inventory_mode'),
            'detector_backend': LaunchConfiguration('detector_backend'),
            'model_path': LaunchConfiguration('model_path'),
            'layout_path': str(share / 'config' / 'qr_shelf_layout.yaml'),
            'confidence_threshold': ParameterValue(
                LaunchConfiguration('confidence_threshold'), value_type=float),
            'confirm_frames': ParameterValue(
                LaunchConfiguration('confirm_frames'), value_type=int),
            'qr_timeout': ParameterValue(
                LaunchConfiguration('qr_timeout'), value_type=float),
            'laser_alignment_threshold': ParameterValue(
                LaunchConfiguration('laser_alignment_threshold'),
                value_type=float),
            'visualization': ParameterValue(
                LaunchConfiguration('visualization'), value_type=bool),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}])
    controller = Node(
        package='uav_control', executable='vision_offboard_controller',
        name='vision_offboard_controller', output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sitl'"])),
        parameters=[{
            'task_mode': 'qr_shelf',
            'target_qr_id': ParameterValue(
                LaunchConfiguration('target_qr_id'), value_type=int),
            'simulation_mode': ParameterValue(
                LaunchConfiguration('simulation_mode'), value_type=bool),
            'enable_offboard': ParameterValue(
                LaunchConfiguration('enable_offboard'), value_type=bool),
            'enable_auto_arm': ParameterValue(
                LaunchConfiguration('enable_auto_arm'), value_type=bool),
            'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)}])
    sitl_condition = IfCondition(
        PythonExpression(["'", mode, "' == 'sitl'"]))
    down_detector = Node(
        package='uav_vision', executable='gazebo_red_target_detector_node',
        name='down_red_target_detector',
        condition=sitl_condition,
        parameters=[str(share / 'config' / 'dual_camera_simulation.yaml')])
    selector = Node(
        package='uav_vision', executable='camera_selector_node',
        name='camera_selector_node', condition=sitl_condition,
        parameters=[
            str(share / 'config' / 'dual_camera_simulation.yaml'),
            {'front_area_threshold': 6000.0,
             'front_size_threshold': 75.0}])
    target_filter = Node(
        package='uav_vision', executable='target_filter_node',
        name='target_filter_node', condition=sitl_condition,
        parameters=[str(share / 'config' / 'dual_camera_simulation.yaml')])
    servo = Node(
        package='uav_vision', executable='visual_servo_node',
        name='visual_servo_node', condition=sitl_condition,
        parameters=[str(share / 'config' / 'dual_camera_simulation.yaml')])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='qr_shelf_camera_bridge',
        condition=IfCondition(PythonExpression(
            ["'", mode, "' != 'offline'"])),
        arguments=[
            '/camera/front/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/down/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        remappings=[
            ('/camera/front/image', '/camera/front/image_raw'),
            ('/camera/down/image', '/camera/down/image_raw')])
    return LaunchDescription(arguments + [
        bridge, detector, down_detector, selector, target_filter, servo,
        controller])
