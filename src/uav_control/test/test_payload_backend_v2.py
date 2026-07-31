"""Payload backend safety tests without accessing GPIO."""

import pytest

from uav_control.payload_backend import (DryRunPayloadBackend, GpioPayloadBackend,
                                         ReleaseConfig, validate_release_config)


def test_dry_run_never_reports_physical_action():
    result, physical = DryRunPayloadBackend().execute(ReleaseConfig())
    assert result == 'DRY_RUN_CONFIRMED'
    assert physical is False


def test_gpio_backend_is_fail_closed():
    with pytest.raises(RuntimeError):
        GpioPayloadBackend(False, True)


def test_release_parameters_are_range_checked():
    with pytest.raises(ValueError):
        validate_release_config(ReleaseConfig(release_pulse_us=3000))
