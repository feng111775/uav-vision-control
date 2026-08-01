from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([FindPackageShare("servo_control"), "config", "servo_dry_run.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config,
                              description="servo parameter file; choose servo_real.yaml explicitly for hardware"),
        Node(package="servo_control", executable="servo_node", name="servo_node",
             output="screen", parameters=[LaunchConfiguration("config")]),
    ])
