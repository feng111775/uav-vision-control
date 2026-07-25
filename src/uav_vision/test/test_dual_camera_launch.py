"""Static wiring tests for the dual-camera simulation launch."""

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node


LAUNCH_PATH = (
    Path(__file__).parents[1] / 'launch'
    / 'dual_camera_simulation.launch.py')
SPEC = importlib.util.spec_from_file_location(
    'dual_camera_simulation_launch', LAUNCH_PATH)
dual_camera_launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dual_camera_launch)


def test_launch_starts_one_formal_controller_and_two_detectors():
    """Launch must contain the requested nodes without offboard_control."""
    description = dual_camera_launch.generate_launch_description()
    nodes = [
        entity for entity in description.entities
        if isinstance(entity, Node)
    ]
    executables = [
        ''.join(str(value) for value in node.node_executable)
        for node in nodes
    ]
    assert executables.count('gazebo_red_target_detector_node') == 2
    assert executables.count('camera_selector_node') == 1
    assert executables.count('target_filter_node') == 1
    assert executables.count('visual_servo_node') == 1
    assert executables.count('vision_offboard_controller') == 1
    assert 'offboard_control' not in executables


def test_launch_exposes_safe_control_arguments():
    """Control arguments must exist and default to disabled."""
    description = dual_camera_launch.generate_launch_description()
    arguments = [
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    defaults = {
        argument.name: ''.join(
            value.text for value in argument.default_value)
        for argument in arguments
    }
    assert defaults == {
        'selector_mode': 'auto',
        'enable_offboard': 'false',
        'enable_auto_arm': 'false',
    }

    controller = next(
        entity for entity in description.entities
        if isinstance(entity, Node)
        and ''.join(str(value) for value in entity.node_executable)
        == 'vision_offboard_controller')
    overrides = {
        ''.join(part.text for part in key): value
        for key, value in controller._Node__parameters[-1].items()
    }
    assert overrides['simulation_mode'] is True
    assert overrides['enable_offboard'].value_type is bool
    assert overrides['enable_auto_arm'].value_type is bool


def test_yaml_wires_only_selected_detection_to_filter():
    """Raw camera detections must meet only at the selector."""
    config = (
        Path(__file__).parents[1] / 'config'
        / 'dual_camera_simulation.yaml').read_text()
    assert 'detection_topic: /vision/front/detection' in config
    assert 'detection_topic: /vision/down/detection' in config
    assert 'selected_detection_topic: /vision/selected_detection' in config
    assert 'detection_topic: /vision/selected_detection' in config
    assert '/vision/h7/detection' not in config
