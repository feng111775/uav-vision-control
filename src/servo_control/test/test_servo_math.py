import pytest

from servo_control.servo_node import ServoConfig, angle_to_pulsewidth


def test_angle_to_pulsewidth_endpoints_and_middle():
    config = ServoConfig(min_pulse_us=1000, max_pulse_us=2000)
    assert angle_to_pulsewidth(0.0, config) == 1000
    assert angle_to_pulsewidth(90.0, config) == 1500
    assert angle_to_pulsewidth(180.0, config) == 2000


@pytest.mark.parametrize("angle", [-1.0, 181.0])
def test_angle_outside_limits_is_rejected(angle):
    with pytest.raises(ValueError):
        angle_to_pulsewidth(angle, ServoConfig())
