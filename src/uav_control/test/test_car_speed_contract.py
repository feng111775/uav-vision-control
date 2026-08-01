"""Keep the competition car-speed contract at 0.1 m/s."""

from pathlib import Path

import yaml


def test_stage4c_car_speed_is_point_one_mps():
    config_dir = Path(__file__).parents[1] / 'config'
    for name in ('mission_stage4c.yaml', 'mission_stage4c_sitl.yaml',
                 'mission_stage4c_hardware_bench.yaml'):
        params = yaml.safe_load((config_dir / name).read_text())['/**'][
            'ros__parameters']
        assert params['car_speed_mps'] == 0.1


def test_no_car_speed_document_uses_point_zero_one():
    root = Path(__file__).parents[2]
    text = '\n'.join(path.read_text(errors='ignore') for path in (
        root / 'docs').rglob('*.md'))
    assert '0.01 m/s' not in text
    assert '0.01m/s' not in text
