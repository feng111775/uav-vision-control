import pytest

from uav_vision.h7_bridge_node import (
    is_diagnostic_line, parse_detection_line, parse_status_line, parse_v2_line)


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


def test_fps_diagnostic_is_not_treated_as_protocol_data():
    assert is_diagnostic_line('D_VISION,status=CROSS_INVALID,fps=3.4')
    assert is_diagnostic_line('D_TIMING,find_circles,avg_ms=289.449')
    assert not is_diagnostic_line('D_TARGET,0,0,0,0,0,0,0')

def test_v2_target_is_parsed_and_diagnostics_are_legal():
    item = parse_v2_line('D_TARGET_V2,7,100,1200,SEARCH,1,160,120,50,30,0.2,80')
    assert item['sequence'] == 7 and item['ticks'] == 100
    for prefix in ('D_BOOT_V2,', 'D_CONFIG,', 'D_STATUS_V2,', 'D_DETECT_STATS,',
                   'D_TIMING,', 'D_VISION,', 'D_ERROR,'):
        assert is_diagnostic_line(prefix + 'x')
    assert is_diagnostic_line('D_VISION_ERROR')
    assert is_diagnostic_line('D_VISION_ERROR,stage=verify')
    assert is_diagnostic_line('D_BOOT_V2')
    assert is_diagnostic_line('D_CONFIG,')

def test_malformed_v2_is_rejected():
    import pytest
    with pytest.raises(ValueError):
        parse_v2_line('D_TARGET_V2,broken')
