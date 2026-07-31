"""Strict local tests for the STM32-to-UDP wire protocol."""

import pytest

from uav_control.car_protocol import parse_stm32_frame, xor_checksum


def frame(payload, suffix='\r\n'):
    return f'${payload}*{xor_checksum(payload):02X}{suffix}'


def test_checksum_and_valid_event_and_car():
    assert xor_checksum('EVT,1,START') == 0x36
    assert parse_stm32_frame(frame('EVT,1,START')).run_id == 1
    parsed = parse_stm32_frame(
        frame('CAR,1,1,100,500,0A,0.25,100,120,FF'))
    assert parsed.kind == 'CAR'
    assert parsed.fields[5] == '0A'


@pytest.mark.parametrize('payload', [
    'EVT,1,UNKNOWN', 'EVT,1', 'CAR,1,5,1,1,00,0,0,0,00',
    'CAR,1,1,1,1001,00,0,0,0,00',
    'CAR,1,1,1,1,GG,0,0,0,00',
    'CAR,1,1,1,1,00,nan,0,0,00',
    'CAR,-1,1,1,1,00,0,0,0,00',
])
def test_invalid_fields_are_rejected(payload):
    with pytest.raises(ValueError):
        parse_stm32_frame(frame(payload))


@pytest.mark.parametrize('raw', [
    '', 'EVT,1,START', '$EVT,1,START*00\r\n',
    '$EVT,1,START*ZZ\r\n', '$EVT,1,START*36extra\r\n',
    '$EVT,1,START*36\r\n$EVT,2,START*36\r\n',
    '$EVT,1,START*36\x00\r\n', '非ASCII',
])
def test_invalid_framing_is_rejected(raw):
    with pytest.raises(ValueError):
        parse_stm32_frame(raw)
