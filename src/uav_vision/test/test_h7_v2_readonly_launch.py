import importlib.util
from pathlib import Path

from launch_ros.actions import Node

PATH = Path(__file__).parents[1] / 'launch' / 'h7_v2_readonly.launch.py'
SPEC = importlib.util.spec_from_file_location('h7_v2_readonly_launch', PATH)
MODULE = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(MODULE)


def test_readonly_launch_has_exactly_two_nodes_and_safe_topics():
    actions = MODULE.generate_launch_description().entities
    nodes = [action for action in actions if isinstance(action, Node)]
    assert [node.node_executable for node in nodes] == ['h7_bridge_node', 'vision_interface_node']
    source = PATH.read_text(encoding='utf-8')
    for forbidden in ('target_filter_node', 'visual_servo_node', 'uav_control',
                      'real_practise', 'MicroXRCEAgent', 'PX4', '/fmu/in'):
        assert forbidden not in source
