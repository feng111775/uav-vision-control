# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Payload actuator wire protocol and deterministic action tracker."""
from dataclasses import dataclass
from enum import IntEnum

from .car_link_protocol import crc16_ccitt


class PayloadProtocolError(ValueError):
    """Invalid payload frame or state transition."""


class PayloadResult(IntEnum):  # noqa: D101
    SUCCESS = 0
    BUSY = 1
    MECHANICAL_ERROR = 2
    SENSOR_ERROR = 3
    DENIED = 4
    UNKNOWN_ERROR = 5


@dataclass(frozen=True)
class PayloadAck:  # noqa: D101
    sequence: int
    result: PayloadResult


def _with_crc(body: str) -> str:
    return f"{body},{crc16_ccitt(body.encode('ascii')):04X}"


def encode_drop(sequence: int) -> str:  # noqa: D103
    if not 0 <= sequence <= 0xFFFF:
        raise PayloadProtocolError("sequence out of range")
    return _with_crc(f"DROP,1,{sequence}")


def encode_drop_ack(ack: PayloadAck) -> str:  # noqa: D103
    return _with_crc(f"DROP_ACK,1,{ack.sequence},{int(ack.result)}")


def decode_drop_ack(line: str | bytes) -> PayloadAck:  # noqa: D103
    try:
        text = line.decode("ascii") if isinstance(line, bytes) else line
        fields = text.strip().split(",")
        if len(fields) != 5 or fields[0] != "DROP_ACK" or fields[1] != "1":
            raise PayloadProtocolError("invalid ack header/version")
        body = ",".join(fields[:-1])
        if int(fields[-1], 16) != crc16_ccitt(body.encode("ascii")):
            raise PayloadProtocolError("CRC mismatch")
        sequence = int(fields[2])
        result = PayloadResult(int(fields[3]))
    except (UnicodeDecodeError, ValueError) as exc:
        raise PayloadProtocolError("invalid ack") from exc
    if not 0 <= sequence <= 0xFFFF:
        raise PayloadProtocolError("sequence out of range")
    return PayloadAck(sequence, result)


class PayloadActionTracker:
    """One rising edge creates one finite-retry actuator transaction."""

    def __init__(self, timeout_seconds: float = 1.0, retry_count: int = 2) -> None:  # noqa: D107
        self.timeout_seconds = timeout_seconds
        self.retry_count = retry_count
        self.sequence = 0
        self.active_sequence: int | None = None
        self.sent_at = 0.0
        self.attempts = 0
        self.input_high = False
        self.status = "IDLE"

    def on_release(self, value: bool, now: float, enabled: bool) -> str | None:  # noqa: D102
        if not value:
            self.input_high = False
            return None
        if self.input_high:
            return None
        self.input_high = True
        if not enabled or self.active_sequence is not None:
            self.status = "DISABLED" if not enabled else "BUSY"
            return None
        self.sequence = (self.sequence + 1) & 0xFFFF
        self.active_sequence = self.sequence
        self.attempts = 1
        self.sent_at = now
        self.status = "WAIT_ACK"
        return encode_drop(self.sequence)

    def poll(self, now: float) -> str | None:  # noqa: D102
        if self.active_sequence is None or now - self.sent_at < self.timeout_seconds:
            return None
        if self.attempts > self.retry_count:
            self.status = "TIMEOUT"
            self.active_sequence = None
            return None
        self.attempts += 1
        self.sent_at = now
        return encode_drop(self.active_sequence)

    def on_ack(self, ack: PayloadAck) -> bool:  # noqa: D102
        if self.active_sequence is None or ack.sequence != self.active_sequence:
            self.status = "STALE_ACK"
            return False
        self.active_sequence = None
        self.status = ack.result.name
        return ack.result == PayloadResult.SUCCESS
