"""Tests for safe camera pipeline launch argument handling."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import LogInfo


LAUNCH_PATH = (
    Path(__file__).parents[1] / 'launch' / 'pi_camera_pipeline.launch.py')
SPEC = importlib.util.spec_from_file_location('pi_camera_pipeline', LAUNCH_PATH)
pi_camera_pipeline = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(pi_camera_pipeline)
camera_node = pi_camera_pipeline.camera_node


def context_with(arguments):
    """Create a launch context containing the requested argument values."""
    context = LaunchContext()
    context.launch_configurations.update(arguments)
    return context


def test_empty_default_video_path_does_not_start_failing_camera():
    """An empty video path should produce diagnostics instead of a Node."""
    actions = camera_node(context_with({
        'source_type': 'video',
        'source': '',
        'device': '0',
    }), '/tmp/config.yaml')
    assert len(actions) == 1
    assert isinstance(actions[0], LogInfo)


def test_launch_arguments_override_camera_yaml():
    """Source arguments must be passed after YAML as explicit overrides."""
    context = context_with({
        'source_type': 'usb',
        'source': '/ignored/by/usb',
        'device': '2',
    })
    with patch.object(pi_camera_pipeline, 'Node') as node_type:
        camera_node(context, '/tmp/config.yaml')

    parameters = node_type.call_args.kwargs['parameters']
    assert parameters == [
        '/tmp/config.yaml',
        {
            'source_type': 'usb',
            'source': '/ignored/by/usb',
            'device': 2,
        },
    ]
