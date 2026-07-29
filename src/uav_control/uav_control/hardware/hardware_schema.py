# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""
Validation shared by all hardware adapters.

An absent physical device is represented by ``transport=disabled``. Empty
ports and zero baud rates are deliberately invalid for active transports.
"""
from dataclasses import dataclass
from typing import Any, Mapping


TRANSPORTS = frozenset({"disabled", "mock", "serial", "udp", "replay"})


class HardwareConfigurationError(ValueError):
    """A hardware adapter configuration is unsafe or incomplete."""


class UnsupportedTransport(HardwareConfigurationError):
    """The requested transport has no implementation for this adapter."""


@dataclass(frozen=True)
class HardwareConfig:  # noqa: D101
    transport: str = "disabled"
    device: str = ""
    baudrate: int = 0
    bind_address: str = ""
    bind_port: int = 0
    allowed_remote_ip: str = ""
    replay_file: str = ""
    timeout_seconds: float = 1.0
    retry_count: int = 2
    crc_type: str = "crc16-ccitt"
    protocol_version: int = 1
    allow_actions: bool = False
    enforce_device_identity: bool = False
    expected_vid: str = ""
    expected_pid: str = ""
    expected_serial: str = ""


def validate_hardware_config(
    values: Mapping[str, Any],
    *,
    supported: set[str] | frozenset[str] = TRANSPORTS,
    action_adapter: bool = False,
) -> HardwareConfig:
    """Return a validated immutable configuration."""
    cfg = HardwareConfig(**{
        key: values[key] for key in HardwareConfig.__dataclass_fields__
        if key in values and values[key] is not None
    })
    if cfg.transport not in TRANSPORTS:
        raise HardwareConfigurationError("unknown transport: " + cfg.transport)
    if cfg.transport not in supported:
        raise UnsupportedTransport(cfg.transport)
    if cfg.protocol_version != 1 or cfg.crc_type.lower() != "crc16-ccitt":
        raise HardwareConfigurationError("only protocol 1 with CRC16-CCITT is supported")
    if cfg.timeout_seconds <= 0.0 or cfg.retry_count < 0:
        raise HardwareConfigurationError("timeout must be positive and retries non-negative")
    if cfg.transport == "serial" and (not cfg.device or cfg.baudrate <= 0):
        raise HardwareConfigurationError("serial requires explicit device and baudrate")
    if cfg.transport == "udp" and (
        not cfg.bind_address or not 1 <= cfg.bind_port <= 65535
    ):
        raise HardwareConfigurationError("udp requires explicit bind address and port")
    if cfg.transport == "replay" and not cfg.replay_file:
        raise HardwareConfigurationError("replay requires an explicit file")
    if cfg.enforce_device_identity and not (
        cfg.expected_vid and cfg.expected_pid and cfg.expected_serial
    ):
        raise HardwareConfigurationError("identity enforcement requires VID, PID, and serial")
    if action_adapter and cfg.allow_actions and cfg.transport == "disabled":
        raise HardwareConfigurationError("disabled transport cannot execute actions")
    return cfg
