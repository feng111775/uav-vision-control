"""Fail-closed payload backend boundary for desktop and Raspberry Pi review."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ReleaseConfig:
    gpio: int = 18
    pwm_frequency_hz: int = 50
    neutral_pulse_us: int = 1500
    release_pulse_us: int = 1900
    hold_seconds: float = 0.8
    reset_seconds: float = 0.8


def validate_release_config(config):
    if not 0 <= int(config.gpio) <= 40:
        raise ValueError('invalid Raspberry Pi GPIO')
    if int(config.pwm_frequency_hz) != 50:
        raise ValueError('servo PWM must be 50 Hz')
    if not (500 <= int(config.neutral_pulse_us) <= 2500 and
            500 <= int(config.release_pulse_us) <= 2500):
        raise ValueError('servo pulse outside safe range')
    if float(config.hold_seconds) <= 0.0 or float(config.reset_seconds) <= 0.0:
        raise ValueError('servo timing must be positive')
    return config


class DryRunPayloadBackend:
    """Software-only backend; it never imports or touches GPIO."""

    name = 'dry_run'

    def execute(self, config, requested='SUCCESS'):
        validate_release_config(config)
        return ('DRY_RUN_CONFIRMED' if str(requested).upper() == 'SUCCESS'
                else 'FAILED', False)


class GpioPayloadBackend:
    """Reserved hardware backend; fail closed until board review is complete."""

    name = 'gpio'

    def __init__(self, simulation_mode, enable_payload_release):
        if simulation_mode or not enable_payload_release:
            raise RuntimeError('GPIO backend requires explicit real enable')
        raise RuntimeError('GPIO backend is not implemented; no PWM was emitted')
