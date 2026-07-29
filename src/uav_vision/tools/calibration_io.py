"""Validated calibration YAML I/O and metric-output safety gates."""

from pathlib import Path

import yaml


def save_yaml(path, data):
    Path(path).write_text(
        yaml.safe_dump(data, sort_keys=False), encoding='utf-8')


def load_yaml(path):
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('calibration YAML must contain a mapping')
    return data


def require_metric_calibration(data):
    if not data.get('camera_calibrated') or not data.get('scale_calibrated'):
        raise RuntimeError(
            'metric output forbidden until camera and scale are calibrated')
    return True
