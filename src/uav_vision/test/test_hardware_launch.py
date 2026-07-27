"""Static safety checks for the first-flight hardware launch."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


LAUNCH_PATH = (
    Path(__file__).parents[1] / 'launch' / 'red_target_hardware.launch.py')
SPEC = importlib.util.spec_from_file_location(
    'red_target_hardware_launch', LAUNCH_PATH)
hardware_launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(hardware_launch)


def test_hardware_launch_has_both_sources_and_one_controller():
    """Hardware launch must wire Pi front, H7 down, and one PX4 controller."""
    actions = hardware_launch.generate_launch_description().entities
    nodes = [action for action in actions if isinstance(action, Node)]
    executables = [node.node_executable for node in nodes]
    assert executables.count('pi_camera_vision_node') == 1
    assert executables.count('h7_bridge_node') == 1
    assert executables.count('vision_offboard_controller') == 1


def test_hardware_launch_defaults_to_monitor_only():
    """First-flight launch must require explicit Offboard enablement."""
    actions = hardware_launch.generate_launch_description().entities
    arguments = {
        action.name: action.default_value[0].text
        for action in actions if isinstance(action, DeclareLaunchArgument)
    }
    assert arguments['enable_offboard'] == 'false'


def test_hardware_yaml_has_required_safe_defaults():
    """Hardware safety values stay separate from simulation configuration."""
    config = (
        Path(__file__).parents[1]
        / 'config' / 'red_target_hardware.yaml').read_text()
    assert 'simulation_mode: false' in config
    assert 'enable_offboard: false' in config
    assert 'enable_auto_arm: false' in config
    assert 'target_altitude: 0.8' in config
    assert 'max_xy_speed: 0.25' in config
    assert 'max_z_speed: 0.20' in config
    assert 'image_width: 320.0' in config
    assert 'image_height: 240.0' in config
