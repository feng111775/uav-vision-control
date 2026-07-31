"""Tests for strict car-frame parsing and gateway safety contracts."""

from uav_control.car_protocol import checksum, parse_frame


def frame(payload):
    return '$%s*%s\r\n' % (payload, checksum(payload))


def test_valid_start_and_car_frames():
    assert parse_frame(frame('EVT,1,START')) == {
        'kind': 'EVT', 'run_id': 1, 'event': 'START'}
    car = parse_frame(frame('CAR,1,1,100,20,18,0,600,600,00'))
    assert car['progress_permille'] == 20
    assert car['line_mask'] == 0x18


def test_bad_checksum_and_unknown_event_are_rejected_or_ignored():
    try:
        parse_frame('$EVT,1,START*00\r\n')
    except ValueError:
        pass
    else:
        raise AssertionError('bad checksum accepted')
    assert parse_frame(frame('EVT,1,UNKNOWN'))['event'] == 'UNKNOWN'
