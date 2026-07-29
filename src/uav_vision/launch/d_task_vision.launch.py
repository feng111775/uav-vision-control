"""Launch the hardware-independent 2026 D-task vision chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _nodes(context):
    source = LaunchConfiguration('input_source').perform(context)
    if source not in ('fake', 'h7'):
        raise RuntimeError('input_source must be fake or h7')
    config = os.path.join(
        get_package_share_directory('uav_vision'), 'config',
        'd_task_vision.yaml')
    executables = [
        'fake_h7_node' if source == 'fake' else 'h7_bridge_node',
        'target_filter_node', 'target_predictor_node',
        'landing_error_node', 'vision_dashboard_node']
    return [Node(package='uav_vision', executable=executable,
                 name=executable, parameters=[config], output='screen')
            for executable in executables]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_source', default_value='fake',
                              description='Exactly one source: fake or h7'),
        OpaqueFunction(function=_nodes),
    ])
