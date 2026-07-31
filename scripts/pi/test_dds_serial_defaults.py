"""Static regression checks for the verified PX4 GPS2 DDS serial link."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_agent_defaults_and_command():
    script = (ROOT / "scripts/pi/start_microxrce_agent.sh").read_text()
    assert 'transport="serial"' in script
    assert 'device="/dev/ttyAMA0"' in script
    assert 'baudrate="921600"' in script
    assert 'serial --dev "$device" -b "$baudrate" -v 4' in script


def test_systemd_defaults_and_no_formal_460800():
    env = (ROOT / "deploy/systemd/d-task.env.example").read_text()
    assert "XRCE_DEVICE=/dev/ttyAMA0" in env
    assert "XRCE_BAUDRATE=921600" in env
    assert "460800" not in env


def test_real_hardware_safety_defaults_unchanged():
    config = (ROOT / "src/uav_control/config/competition_drop.yaml").read_text()
    assert "simulation_mode: false" in config
    assert "enable_auto_arm: false" in config
    assert "enable_payload_release: false" in config
    assert "closed_loop_enable: false" in (
        ROOT / "src/uav_vision/config/d_task_vision.yaml"
    ).read_text()
