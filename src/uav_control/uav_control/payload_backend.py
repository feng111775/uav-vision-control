"""
Pluggable payload backends with a fail-closed GPIO boundary.

No GPIO library is imported here.  The Raspberry Pi implementation can be
added after the board, library and electrical interlock are confirmed.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ReleaseConfig:
    gpio: int = 18
    pwm_frequency_hz: int = 50
    neutral_pulse_us: int = 1500
    release_pulse_us: int = 1900
    hold_seconds: float = 0.8
    reset_seconds: float = 0.8
    min_pulse_us: int = 500
    max_pulse_us: int = 2500


def validate_release_config(config: ReleaseConfig):
    """Validate safe servo timing without touching GPIO."""
    if not 0 <= int(config.gpio) <= 40:
        raise ValueError('gpio must be a Raspberry Pi GPIO number in [0, 40]')
    if int(config.pwm_frequency_hz) != 50:
        raise ValueError('only 50 Hz servo PWM is supported')
    if not 300 <= int(config.min_pulse_us) < int(config.max_pulse_us) <= 3000:
        raise ValueError('servo pulse bounds are invalid')
    for value in (config.neutral_pulse_us, config.release_pulse_us):
        if not int(config.min_pulse_us) <= int(value) <= int(config.max_pulse_us):
            raise ValueError('servo pulse is outside configured bounds')
    if float(config.hold_seconds) <= 0.0 or float(config.reset_seconds) <= 0.0:
        raise ValueError('servo hold/reset times must be positive')
    return config


class DryRunPayloadBackend:
    """Backend used by tests; it never opens a device or emits PWM."""

    name = 'dry_run'

    def execute(self, config, requested_result='SUCCESS'):
        validate_release_config(config)
        result = str(requested_result).upper()
        return ('DRY_RUN_CONFIRMED' if result == 'SUCCESS' else 'FAILED', False)


class GpioPayloadBackend:
    """Reserved Raspberry Pi backend; fail closed until reviewed implementation."""

    name = 'gpio'

    def __init__(self, simulation_mode, enable_payload_release):
        if simulation_mode:
            raise RuntimeError('gpio backend requires simulation_mode=false')
        if not enable_payload_release:
            raise RuntimeError('gpio backend requires enable_payload_release=true')
        raise RuntimeError(
            'GPIO backend is not implemented; confirm the board library and wiring first')
