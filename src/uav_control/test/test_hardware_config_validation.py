# flake8: noqa
"""Formal configuration boundary tests."""
from pathlib import Path
import importlib.util

import yaml

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "validator", ROOT / "scripts/config/validate_d_task_config.py"
)
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


def errors(data, name="test.yaml"):
    return validator.validate_data(data, name)


def test_mock_rejected_in_competition():
    assert errors({"payload": {"transport": "mock"}}, "competition_drop.yaml")


def test_openmv_ttyacm0_rejected(tmp_path):
    path = tmp_path / "openmv.yaml"
    path.write_text("openmv:\n  device: /dev/ttyACM0\n")
    assert validator.validate_file(path)


def test_serial_missing_values_rejected():
    assert errors({"car": {"transport": "serial"}})


def test_udp_missing_port_rejected():
    assert errors({"car": {"transport": "udp", "bind_address": "127.0.0.1"}})


def test_first_flight_boundaries():
    data = yaml.safe_load(
        (ROOT / "src/uav_control/config/first_flight_hover.yaml").read_text()
    )
    assert not errors(data, "first_flight_hover.yaml")


def test_px4_topic_constants():
    assert validator.LOCAL_POSITION == "/fmu/out/vehicle_local_position"
    assert validator.VEHICLE_STATUS == "/fmu/out/vehicle_status_v1"


def test_competition_defaults_control_off():
    for name in ("competition_drop.yaml", "competition_dynamic_land.yaml"):
        data = yaml.safe_load((ROOT / "src/uav_control/config" / name).read_text())
        assert not errors(data, name)


def test_unknown_gpio_rejected(tmp_path):
    path = tmp_path / "hardware.yaml"
    path.write_text("payload:\n  gpio_pin: 17\n")
    assert validator.validate_file(path)
