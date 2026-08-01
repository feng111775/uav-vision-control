"""Pure software protocol and journal helpers for the car UDP gateway."""

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import tempfile
import time


MAX_RUN_ID = 0xFFFFFFFF
MAX_FRAME_BYTES = 512
VALID_EVENTS = {
    'BOOT', 'READY', 'START', 'FINISH', 'TIMEOUT', 'ESTOP', 'REMOTE_STOP',
    'PONG'}
VALID_CAR_STATES = {0, 1, 2, 3, 4}


@dataclass(frozen=True)
class ParsedFrame:
    kind: str
    run_id: int
    fields: tuple[str, ...]
    raw: str


def xor_checksum(payload: str) -> int:
    """Calculate ASCII XOR over the payload without ``$`` or ``*``."""
    try:
        encoded = payload.encode('ascii')
    except UnicodeEncodeError as error:
        raise ValueError('frame payload must be ASCII') from error
    checksum = 0
    for value in encoded:
        checksum ^= value
    return checksum


def parse_stm32_frame(raw_text: str, max_bytes=MAX_FRAME_BYTES) -> ParsedFrame:
    """Parse exactly one ``$...*HH`` datagram and validate its fields."""
    if not isinstance(raw_text, str) or not raw_text:
        raise ValueError('empty frame')
    if '\x00' in raw_text or '\n' in raw_text[:-1]:
        raise ValueError('frame contains NUL or multiple lines')
    text = raw_text[:-1] if raw_text.endswith('\n') else raw_text
    if text.endswith('\r'):
        text = text[:-1]
    try:
        encoded_text = text.encode('ascii')
    except UnicodeEncodeError as error:
        raise ValueError('frame must be ASCII') from error
    if len(encoded_text) > int(max_bytes):
        raise ValueError('frame exceeds maximum size')
    if not text.startswith('$'):
        raise ValueError("missing '$' frame start")
    star_index = text.find('*')
    if star_index <= 1 or text.find('*', star_index + 1) != -1:
        raise ValueError('invalid checksum separator')
    payload = text[1:star_index]
    checksum_text = text[star_index + 1:]
    if len(checksum_text) != 2:
        raise ValueError('checksum must contain exactly two hex digits')
    try:
        received = int(checksum_text, 16)
    except ValueError as error:
        raise ValueError('checksum is not hexadecimal') from error
    calculated = xor_checksum(payload)
    if received != calculated:
        raise ValueError('checksum mismatch')
    fields = tuple(payload.split(','))
    if fields[0] == 'EVT' and len(fields) == 3:
        kind = 'EVT'
        if fields[2].upper() not in VALID_EVENTS:
            raise ValueError('unknown EVT event')
    elif fields[0] == 'CAR' and len(fields) == 10:
        kind = 'CAR'
    else:
        raise ValueError('invalid frame kind or field count')
    try:
        run_id = int(fields[1], 10)
    except ValueError as error:
        raise ValueError('run_id is not decimal') from error
    if not 0 <= run_id <= MAX_RUN_ID:
        raise ValueError('run_id outside uint32 range')
    if kind == 'CAR':
        try:
            state = int(fields[2], 10)
            elapsed = int(fields[3], 10)
            progress = int(fields[4], 10)
            line_mask = int(fields[5], 16)
            line_error = float(fields[6])
            left_pwm = int(fields[7], 10)
            right_pwm = int(fields[8], 10)
            flags = int(fields[9], 16)
        except ValueError as error:
            raise ValueError('invalid CAR numeric field') from error
        if (state not in VALID_CAR_STATES or elapsed < 0 or
                not 0 <= progress <= 1000 or not math.isfinite(line_error) or
                not 0 <= line_mask <= 0xFF or
                not 0 <= flags <= 0xFF or
                not -1000 <= left_pwm <= 1000 or
                not -1000 <= right_pwm <= 1000):
            raise ValueError('CAR numeric field outside range')
    return ParsedFrame(kind, run_id, fields, text)


class StartJournal:
    """Atomic pending/committed journal with fail-closed recovery."""

    def __init__(self, path):
        self.path = Path(path).expanduser()
        self.pending_run_id = None
        self.pending_source = None
        self.pending_received_monotonic = None
        self.committed_run_id = None
        self.locked = False
        self._load()

    def _load(self):
        if not self.path.exists():
            return
        try:
            data = json.loads(self.path.read_text(encoding='utf-8'))
            if data.get('schema_version') != 1:
                raise ValueError('unsupported journal schema')
            pending = data.get('pending_run_id')
            committed = data.get('committed_run_id')
            for value in (pending, committed):
                if value is not None and not 0 <= int(value) <= MAX_RUN_ID:
                    raise ValueError('journal run_id outside uint32 range')
            self.pending_run_id = None if pending is None else int(pending)
            self.pending_source = data.get('pending_source')
            received = data.get('pending_received_monotonic')
            self.pending_received_monotonic = (
                None if received is None else float(received))
            self.committed_run_id = None if committed is None else int(committed)
        except (OSError, TypeError, ValueError, json.JSONDecodeError):
            self.locked = True
            raise RuntimeError('UDP start journal is unreadable')

    def _write(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix=self.path.name + '.', dir=str(self.path.parent))
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump({
                    'schema_version': 1,
                    'pending_run_id': self.pending_run_id,
                    'pending_source': self.pending_source,
                    'pending_received_monotonic':
                        self.pending_received_monotonic,
                    'committed_run_id': self.committed_run_id,
                }, stream, sort_keys=True)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        except Exception:
            try:
                os.unlink(temporary)
            except OSError:
                pass
            self.locked = True
            raise

    def begin(self, run_id, source):
        if self.locked:
            return 'LOCKED'
        run_id = int(run_id)
        if (self.committed_run_id is not None and
                run_id <= self.committed_run_id):
            return 'ALREADY_PROCESSED'
        if self.pending_run_id is not None:
            return 'BUSY'
        self.pending_run_id = run_id
        self.pending_source = str(source)
        self.pending_received_monotonic = time.monotonic()
        self._write()
        return 'PENDING'

    def commit(self, run_id):
        if self.locked or self.pending_run_id != int(run_id):
            return False
        self.committed_run_id = int(run_id)
        self.pending_run_id = None
        self.pending_source = None
        self.pending_received_monotonic = None
        self._write()
        return True

    def begin_new_session(self):
        """Forget committed run IDs only after an explicit safe session reset."""
        if self.locked or self.pending_run_id is not None:
            return False
        self.committed_run_id = None
        self._write()
        return True
