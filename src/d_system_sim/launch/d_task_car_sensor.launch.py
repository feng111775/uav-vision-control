from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from car_control_launch.robot_description import make_robot_description
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_field_parameters(path):
    values = {}
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if line:
            key, value = line.split(":", 1)
            values[key.strip()] = float(value.strip())
    return values


def generate_launch_description():
    car_share = Path(get_package_share_directory("car_control"))
    sim_share = Path(get_package_share_directory("d_system_sim"))
    world = str(sim_share / "worlds/d_task_field.sdf")
    controllers = str(car_share / "config/car_controllers.yaml")
    sensor_config = str(car_share / "config/virtual_gray_sensor.yaml")
    gui_config = str(sim_share / "config/d_task_gui.config")
    field = load_field_parameters(sim_share / "config/d_task_field.yaml")
    yaw_default = field["car_initial_yaw_rad"]
    front_offset = field["car_front_offset_m"]
    base_x_default = field["left_x_m"] - front_offset * math.cos(yaw_default)
    base_y_default = field["lower_y_m"] - front_offset * math.sin(yaw_default)
    description = make_robot_description(
        car_share / "urdf/car_sim.urdf.xacro", controllers)
    gz_launch = str(Path(get_package_share_directory("ros_gz_sim")) / "launch/gz_sim.launch.py")
    headless = LaunchConfiguration("headless")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": f"-r -s {world}", "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(headless))
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            "gz_args": f"-r --gui-config {gui_config} {world}",
            "on_exit_shutdown": "true",
        }.items(),
        condition=UnlessCondition(headless))
    state_publisher = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": description, "use_sim_time": True}], output="screen")
    spawn = Node(
        package="ros_gz_sim", executable="create",
        arguments=[
            "-name", "car_sim", "-topic", "robot_description",
            "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"), "-Y", LaunchConfiguration("yaw")],
        output="screen")
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}], output="screen")
    spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "joint_state_broadcaster", "diff_drive_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "30", "--service-call-timeout", "10",
            "--switch-timeout", "10"],
        parameters=[{"use_sim_time": True}], output="screen")
    sensor = Node(
        package="car_control", executable="virtual_gray_sensor_node",
        parameters=[
            sensor_config,
            {
                "initial_world_x": LaunchConfiguration("x"),
                "initial_world_y": LaunchConfiguration("y"),
                "initial_world_yaw": LaunchConfiguration("yaw"),
                "sensor_forward_offset_m": front_offset,
                "use_sim_time": True,
            }],
        output="screen")

    def shutdown_on_failure(label):
        def handler(event, _context):
            if event.returncode == 0:
                return []
            return [
                LogInfo(msg=f"ERROR: {label} failed; shutting down."),
                EmitEvent(event=Shutdown(reason=f"{label} failed with code {event.returncode}"))]
        return handler

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("x", default_value=f"{base_x_default:.12f}"),
        DeclareLaunchArgument("y", default_value=f"{base_y_default:.12f}"),
        DeclareLaunchArgument("z", default_value="0.02"),
        DeclareLaunchArgument("yaw", default_value=f"{yaw_default:.16f}"),
        gazebo, gazebo_gui, state_publisher, bridge, spawn, spawner, sensor,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=shutdown_on_failure("ros_gz_sim create"))),
        RegisterEventHandler(OnProcessExit(
            target_action=spawner, on_exit=shutdown_on_failure("controller spawner"))),
    ])
