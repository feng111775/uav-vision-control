from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim_share = Path(get_package_share_directory("d_system_sim"))
    line_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(sim_share / "launch/d_task_car_line_follow.launch.py")),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "auto_start": LaunchConfiguration("auto_start"),
            "stop_after_finish": LaunchConfiguration("stop_after_finish"),
            "initial_base_x": LaunchConfiguration("initial_base_x"),
            "initial_base_y": LaunchConfiguration("initial_base_y"),
            "initial_yaw": LaunchConfiguration("initial_yaw"),
        }.items())
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("stop_after_finish", default_value="true"),
        DeclareLaunchArgument("initial_base_x", default_value="1.50"),
        DeclareLaunchArgument("initial_base_y", default_value="1.80"),
        DeclareLaunchArgument("initial_yaw", default_value="1.5707963267948966"),
        line_follow,
    ])
