import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('real_practise'), 'config', 'first_flight_real.yaml')
    return LaunchDescription([
        Node(package='uav_control', executable='mission_controller_node', name='mission_controller_node',
             parameters=[cfg, {'simulation_mode': False, 'enable_control': True,
                               'enable_auto_arm': False, 'enable_visual_follow': False,
                               'enable_payload_release': False, 'enable_dynamic_landing': False,
                               'enable_second_takeoff': False, 'target_altitude': 0.50,
                               'hover_test_seconds': 3.0}]),
        Node(package='real_practise', executable='first_flight_supervisor_node',
             name='first_flight_supervisor_node', parameters=[cfg, {'simulation_mode': False}]),
    ])
