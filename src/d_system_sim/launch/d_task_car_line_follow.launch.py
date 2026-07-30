from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from d_system_sim.field_config import load_field_parameters


def generate_launch_description():
    sim_share = Path(get_package_share_directory("d_system_sim"))
    car_share = Path(get_package_share_directory("car_control"))
    field = load_field_parameters(sim_share / "config/d_task_field.yaml")
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sim_share / "launch/d_task_car_sensor.launch.py")),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "x": LaunchConfiguration("initial_base_x"),
            "y": LaunchConfiguration("initial_base_y"),
            "yaw": LaunchConfiguration("initial_yaw"),
        }.items())
    controller = Node(
        package="car_control",
        executable="line_follow_controller_node",
        parameters=[
            str(car_share / "config/line_follow_controller.yaml"),
            {
                "use_sim_time": True,
                "auto_start": LaunchConfiguration("auto_start"),
                "initial_base_x": LaunchConfiguration("initial_base_x"),
                "initial_base_y": LaunchConfiguration("initial_base_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
                "stop_after_finish": LaunchConfiguration("stop_after_finish"),
                "progress.a_x": field["left_x_m"],
                "progress.a_y": field["lower_y_m"],
                "progress.b_x": field["left_x_m"],
                "progress.b_y": field["upper_y_m"],
                "progress.c_x": field["right_x_m"],
                "progress.c_y": field["upper_y_m"],
                "progress.d_x": field["right_x_m"],
                "progress.d_y": field["lower_y_m"],
            }],
        output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="false"),
        DeclareLaunchArgument("initial_base_x", default_value="1.50"),
        DeclareLaunchArgument("initial_base_y", default_value="1.80"),
        DeclareLaunchArgument("initial_yaw", default_value="1.5707963267948966"),
        DeclareLaunchArgument("stop_after_finish", default_value="true"),
        sensor_launch,
        controller,
    ])
