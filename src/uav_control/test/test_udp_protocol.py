"""Pure localhost-safe tests for the STM32-to-UDP gateway protocol."""

import json

import pytest

from uav_control.udp_protocol import (
    parse_stm32_frame, StartJournal, xor_checksum)


def frame(payload, lower=False):
    checksum = '%02x' if lower else '%02X'
    return '$%s*%s\r\n' % (payload, checksum % xor_checksum(payload))


def test_evt_and_car_frames_accept_upper_and_lower_checksum():
    event = parse_stm32_frame(frame('EVT,7,START'))
    car = parse_stm32_frame(frame('CAR,7,1,100,500,3,0.25,100,120,0', True))
    assert (event.kind, event.run_id) == ('EVT', 7)
    assert (car.kind, car.run_id, car.fields[4]) == ('CAR', 7, '500')


@pytest.mark.parametrize('text', [
    'EVT,1,START', '$EVT,1,START*00',
    '$EVT,1,START*ZZ\r\n', '$EVT,1,START*AAextra\r\n',
    '$EVT,1,START*AA\r\n$EVT,2,START*AA\r\n',
    '$CAR,1,1,1,100,0,NaN,0,0,0*00',
    '$CAR,-1,1,1,100,0,0,0,0,0*00',
])
def test_malformed_frames_are_rejected(text):
    with pytest.raises(ValueError):
        parse_stm32_frame(text)


def test_frame_ranges_and_single_datagram_limit():
    with pytest.raises(ValueError):
        parse_stm32_frame(frame('EVT,4294967296,START'))
    with pytest.raises(ValueError):
        parse_stm32_frame(frame('CAR,1,1,1,1001,0,0,0,0,0'))
    with pytest.raises(ValueError):
        parse_stm32_frame(frame('EVT,1,START') + 'x')


def test_pending_commit_recovery_and_duplicate_protection(tmp_path):
    path = tmp_path / 'journal.json'
    journal = StartJournal(path)
    assert journal.begin(3, 'EVT_START') == 'PENDING'
    recovered = StartJournal(path)
    assert recovered.pending_run_id == 3
    assert recovered.begin(3, 'EVT_START') == 'BUSY'
    assert recovered.commit(3)
    committed = StartJournal(path)
    assert committed.begin(3, 'CAR_RUNNING_FALLBACK') == 'ALREADY_PROCESSED'
    assert committed.begin(2, 'EVT_START') == 'ALREADY_PROCESSED'
    assert committed.begin(4, 'EVT_START') == 'PENDING'


def test_corrupt_or_invalid_journal_fails_closed(tmp_path):
    path = tmp_path / 'journal.json'
    path.write_text('{broken', encoding='utf-8')
    with pytest.raises(RuntimeError):
        StartJournal(path)
    path.write_text(json.dumps({'schema_version': 99}), encoding='utf-8')
    with pytest.raises(RuntimeError):
        StartJournal(path)


def test_checksum_is_ascii_xor_and_finite_payload_rejected():
    assert xor_checksum('EVT,1,START') == 0x36
    with pytest.raises(ValueError):
        parse_stm32_frame(frame('CAR,1,1,1,100,0,inf,0,0,0'))
