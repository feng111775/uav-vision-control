import pytest

from uav_vision.h7_bridge_node import parse_detection_line, parse_status_line


def test_d_target_serial_parsing():
    assert parse_detection_line(
        'D_TARGET,1,160,120,50,30,-0.2,90') == [
            1.0, 160.0, 120.0, 50.0, 30.0, -0.2, 90.0]


def test_old_target_rejected_by_default():
    with pytest.raises(ValueError):
        parse_detection_line('TARGET,1,160,120,50,30,0.2,90')


def test_legacy_mode_only_accepts_unambiguous_no_target_heartbeat():
    assert parse_detection_line(
        'TARGET,0,0,0,0,0,0,0', True) == [0.0] * 7
    with pytest.raises(ValueError):
        parse_detection_line('TARGET,1,160,120,50,30,0.2,90', True)


@pytest.mark.parametrize('line', [
    'D_TARGET,1,160', 'D_TARGET,2,160,120,50,30,0,90',
    'D_TARGET,1,-1,120,50,30,0,90',
    'D_TARGET,1,160,120,20,30,0,90',
    'D_TARGET,1,160,120,50,30,nan,90'])
def test_bad_lines_rejected(line):
    with pytest.raises(ValueError):
        parse_detection_line(line)


@pytest.mark.parametrize('status', [
    'TRACKING', 'LOST', 'CROSS_INVALID', 'DETECT_ERROR'])
def test_diagnostic_status_protocol(status):
    assert parse_status_line('D_STATUS,' + status) == status


def test_invalid_diagnostic_status_does_not_break_detection_parser():
    with pytest.raises(ValueError):
        parse_status_line('D_STATUS,UNKNOWN')
    assert parse_detection_line(
        'D_TARGET,1,160,120,100,60,0,80')[0] == 1.0
