"""No-GPIO tests for payload backend policy and idempotent interface."""

import pytest

from uav_control.payload_backend import (
    DryRunPayloadBackend, GpioPayloadBackend, ReleaseConfig,
    validate_release_config)


def test_dry_run_backend_never_reports_physical_action():
    result, physical = DryRunPayloadBackend().execute(
        ReleaseConfig(), 'SUCCESS')
    assert result == 'DRY_RUN_CONFIRMED'
    assert physical is False


@pytest.mark.parametrize('field,value', [
    ('gpio', 99), ('pwm_frequency_hz', 60), ('release_pulse_us', 4000),
    ('hold_seconds', 0.0)])
def test_servo_parameters_are_range_checked(field, value):
    values = ReleaseConfig().__dict__
    values[field] = value
    with pytest.raises(ValueError):
        validate_release_config(ReleaseConfig(**values))


def test_gpio_backend_is_fail_closed_without_board_dependency():
    with pytest.raises(RuntimeError, match='not implemented'):
        GpioPayloadBackend(False, True)
