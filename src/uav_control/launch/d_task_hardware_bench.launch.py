# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""No-prop bench launch; every action path is disabled by default."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    car = LaunchConfiguration("car_transport").perform(context)
    payload = LaunchConfiguration("payload_transport").perform(context)
    source = LaunchConfiguration("vision_source").perform(context)
    if car not in {"disabled", "mock"}:
        raise RuntimeError("bench car transport needs an explicit external config")
    if payload not in {"disabled", "mock"}:
        raise RuntimeError("bench payload transport needs an explicit external config")
    control = get_package_share_directory("uav_control")
    vision = get_package_share_directory("uav_vision")
    health = os.path.join(control, "config", "hardware_health.yaml")
    mission = os.path.join(control, "config", "first_flight_hover.yaml")
    result = [
        Node(package="uav_control", executable="mission_controller_node",
             parameters=[mission, {"enable_control": False,
                                   "enable_auto_arm": False}], output="screen"),
        Node(package="uav_control", executable="mission_dashboard_node"),
        Node(package="uav_control", executable="system_health_node",
             parameters=[health]),
        Node(package="uav_control", executable="safety_gate_node",
             parameters=[health, {"mode": "bench"}]),
        Node(package="uav_control", executable="payload_bridge_node",
             parameters=[health, {"transport": payload,
                                  "allow_actions": payload == "mock"}]),
    ]
    if car == "mock":
        result.append(Node(package="uav_control",
                           executable="car_link_simulator_node"))
    else:
        result.append(Node(package="uav_control", executable="car_link_bridge_node",
                           parameters=[health, {"transport": "disabled"}]))
    if payload == "mock":
        result.append(Node(package="uav_control", executable="payload_mock_node"))
    result.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vision, "launch", "d_task_vision.launch.py")),
        launch_arguments={"input_source": source}.items()))
    return result


def generate_launch_description():  # noqa: D103
    return LaunchDescription([
        DeclareLaunchArgument("vision_source", default_value="fake"),
        DeclareLaunchArgument("car_transport", default_value="disabled"),
        DeclareLaunchArgument("payload_transport", default_value="disabled"),
        OpaqueFunction(function=_nodes),
    ])
