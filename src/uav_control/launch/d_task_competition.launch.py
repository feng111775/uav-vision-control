# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Competition wiring with all physical transports unconfigured and disabled."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    mode = LaunchConfiguration("profile").perform(context)
    if mode not in {"drop", "dynamic_land"}:
        raise RuntimeError("profile must be drop or dynamic_land")
    share = get_package_share_directory("uav_control")
    mission = os.path.join(share, "config", f"competition_{mode}.yaml")
    health = os.path.join(share, "config", "hardware_health.yaml")
    return [
        Node(package="uav_control", executable="mission_controller_node",
             parameters=[mission, {"enable_control": False,
                                   "enable_auto_arm": False}]),
        Node(package="uav_control", executable="mission_dashboard_node"),
        Node(package="uav_control", executable="system_health_node",
             parameters=[health]),
        Node(package="uav_control", executable="safety_gate_node",
             parameters=[health, {"mode": mode,
                                  "competition_configured": False}]),
        Node(package="uav_control", executable="car_start_gateway",
             parameters=[{"communication_only": False,
                           "udp_bind_host": "0.0.0.0",
                           "udp_bind_port": 4210}]),
        Node(package="uav_control", executable="payload_bridge_node",
             parameters=[health, {"transport": "disabled",
                                  "allow_actions": False}]),
    ]


def generate_launch_description():  # noqa: D103
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="drop"),
        OpaqueFunction(function=_nodes),
    ])
