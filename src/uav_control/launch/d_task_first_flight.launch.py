# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Only supported first-flight entry; safe defaults cannot command PX4."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():  # noqa: D103
    share = get_package_share_directory("uav_control")
    mission = os.path.join(share, "config", "first_flight_hover.yaml")
    health = os.path.join(share, "config", "hardware_health.yaml")
    return LaunchDescription([
        Node(package="uav_control", executable="mission_controller_node",
             parameters=[mission, {"enable_control": False,
                                   "enable_auto_arm": False,
                                   "enable_visual_follow": False,
                                   "enable_payload_release": False,
                                   "enable_dynamic_landing": False,
                                   "enable_second_takeoff": False}]),
        Node(package="uav_control", executable="mission_dashboard_node"),
        Node(package="uav_control", executable="system_health_node",
             parameters=[health]),
        Node(package="uav_control", executable="safety_gate_node",
             parameters=[health, {"mode": "hover_test",
                                  "competition_configured": False}]),
    ])
