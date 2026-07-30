"""Create one explicitly typed robot_description for all car launch files."""

from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def make_robot_description(xacro_file, controllers_file):
    content = Command([
        FindExecutable(name="xacro"),
        " ",
        str(xacro_file),
        " controllers_file:=",
        str(controllers_file),
    ])
    return ParameterValue(content, value_type=str)
