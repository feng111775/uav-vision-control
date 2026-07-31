"""Strict parser for STM32 car frames forwarded by the Raspberry Pi gateway."""

import re


FRAME_RE = re.compile(r'^\$(?P<payload>[^*\r\n]+)\*(?P<checksum>[0-9A-Fa-f]{2})\r?\n?$')


def checksum(payload):
    value = 0
    for byte in payload.encode('ascii'):
        value ^= byte
    return '%02X' % value


def parse_frame(frame):
    if not isinstance(frame, str):
        raise ValueError('frame must be text')
    try:
        frame.encode('ascii')
    except UnicodeEncodeError as error:
        raise ValueError('frame must be ASCII') from error
    match = FRAME_RE.fullmatch(frame)
    if match is None:
        raise ValueError('invalid frame envelope')
    payload = match.group('payload')
    if checksum(payload) != match.group('checksum').upper():
        raise ValueError('checksum mismatch')
    fields = payload.split(',')
    if fields[0] == 'EVT' and len(fields) == 3:
        run_id = int(fields[1])
        if run_id < 0:
            raise ValueError('run_id must be non-negative')
        return {'kind': 'EVT', 'run_id': run_id, 'event': fields[2]}
    if fields[0] == 'CAR' and len(fields) == 10:
        values = [int(value, 16) if index in (4, 8) else int(value)
                  for index, value in enumerate(fields[1:])]
        run_id, state, elapsed, progress, line_mask, line_error, left, right, flags = values
        if run_id < 0 or state not in range(5) or elapsed < 0 or not 0 <= progress <= 1000:
            raise ValueError('CAR range invalid')
        return {'kind': 'CAR', 'run_id': run_id, 'state': state,
                'elapsed_ms': elapsed, 'progress_permille': progress,
                'line_mask': line_mask, 'line_error': line_error,
                'left_pwm': left, 'right_pwm': right, 'flags': flags}
    raise ValueError('unsupported payload')
