"""Static launch safety and parameter wiring tests."""

from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_qr_launch_has_three_modes_and_safe_defaults():
    text = (ROOT / 'launch' / 'qr_shelf_task.launch.py').read_text()
    assert "choices=['offline', 'observe', 'sitl']" in text
    assert "DeclareLaunchArgument('enable_offboard', default_value='false')" \
        in text
    assert "DeclareLaunchArgument('enable_auto_arm', default_value='false')" \
        in text


def test_all_required_parameters_are_wired():
    text = (ROOT / 'launch' / 'qr_shelf_task.launch.py').read_text()
    for name in ('target_qr_id', 'inventory_mode', 'detector_backend',
                 'model_path', 'confidence_threshold', 'confirm_frames',
                 'qr_timeout', 'laser_alignment_threshold',
                 'enable_offboard', 'enable_auto_arm', 'simulation_mode',
                 'visualization', 'use_sim_time'):
        assert name in text


def test_layout_contains_exactly_24_unique_ids():
    import yaml
    data = yaml.safe_load(
        (ROOT / 'config' / 'qr_shelf_layout.yaml').read_text())
    ids = [entry['id'] for entry in data['qr_codes']]
    assert ids == list(range(1, 25))
    assert data['physical_qr_size_m'] == 0.19
