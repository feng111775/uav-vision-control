# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Transport-independent car telemetry protocol."""
from dataclasses import dataclass

MAX_LINE_BYTES = 256


class CarProtocolError(ValueError):
    """Invalid or unsafe car frame."""


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:  # noqa: D103
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class CarFrame:  # noqa: D101
    sequence: int
    timestamp_ms: int
    progress: int
    flags: int

    @property
    def mission_start(self) -> bool:  # noqa: D102
        return bool(self.flags & 1)


def encode_car_frame(frame: CarFrame) -> str:  # noqa: D103
    _validate(frame)
    body = f"CAR,1,{frame.sequence},{frame.timestamp_ms},{frame.progress},{frame.flags}"
    return f"{body},{crc16_ccitt(body.encode('ascii')):04X}"


def decode_car_frame(data: str | bytes, max_bytes: int = MAX_LINE_BYTES) -> CarFrame:  # noqa: D103
    raw = data if isinstance(data, bytes) else data.encode("utf-8")
    if len(raw) > max_bytes:
        raise CarProtocolError("frame too long")
    try:
        text = raw.decode("ascii").strip()
    except UnicodeDecodeError as exc:
        raise CarProtocolError("frame is not ASCII") from exc
    fields = text.split(",")
    if len(fields) != 7 or fields[0] != "CAR":
        raise CarProtocolError("invalid header or field count")
    if fields[1] != "1":
        raise CarProtocolError("unsupported protocol version")
    body = ",".join(fields[:-1])
    try:
        received_crc = int(fields[-1], 16)
        frame = CarFrame(*(int(value, 10) for value in fields[2:6]))
    except ValueError as exc:
        raise CarProtocolError("invalid numeric field") from exc
    if received_crc != crc16_ccitt(body.encode("ascii")):
        raise CarProtocolError("CRC mismatch")
    _validate(frame)
    return frame


def _validate(frame: CarFrame) -> None:
    if not 0 <= frame.sequence <= 0xFFFF:
        raise CarProtocolError("sequence out of range")
    if not 0 <= frame.timestamp_ms <= 0xFFFFFFFF:
        raise CarProtocolError("timestamp out of range")
    if not 0 <= frame.progress <= 5:
        raise CarProtocolError("progress out of range")
    if not 0 <= frame.flags <= 0xFFFF:
        raise CarProtocolError("flags out of range")


class CarSequenceGuard:
    """Reject duplicates, old frames, and progress regression, including wrap."""

    def __init__(self) -> None:  # noqa: D107
        self.last_sequence: int | None = None
        self.last_progress = 0

    def accept(self, frame: CarFrame) -> None:  # noqa: D102
        if self.last_sequence is not None:
            delta = (frame.sequence - self.last_sequence) & 0xFFFF
            if delta == 0:
                raise CarProtocolError("duplicate sequence")
            if delta > 0x7FFF:
                raise CarProtocolError("out-of-order sequence")
            if frame.progress < self.last_progress:
                raise CarProtocolError("progress regression")
        self.last_sequence = frame.sequence
        self.last_progress = frame.progress
