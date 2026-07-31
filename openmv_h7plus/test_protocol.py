import io
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from protocol import (TargetProtocol, format_target_v2,
                      normalize_result_for_protocol)


def test_none_and_missing_are_zero():
    line = format_target_v2(None, -1, -2, -3, 'BAD')
    assert line == 'D_TARGET_V2,0,0,0,SEARCH,0,0,0,0,0,0.0000,0\n'


def test_invalid_none_nan_and_missing_geometry_clear_coordinates():
    for result in ({'valid': 0}, {'valid': 0, 'cx': None},
                   {'valid': 0, 'cx': float('nan'), 'angle_rad': float('inf')}):
        fields = format_target_v2(result, 1, 2, 3).strip().split(',')
        assert len(fields) == 12 and fields[6:12] == ['0', '0', '0', '0', '0.0000', '0']


def test_valid_geometry_and_confidence_clamped():
    result = {'valid': 1, 'cx': '4', 'cy': 5.9, 'outer_diameter_px': 20,
              'inner_diameter_px': 10, 'angle_rad': .25, 'confidence': 120}
    assert format_target_v2(result, 1, 2, 3, 'FOLLOW').strip().split(',') == [
        'D_TARGET_V2', '1', '2', '3', 'FOLLOW', '1', '4', '5', '20', '10', '0.2500', '100']
    result['confidence'] = -10
    assert format_target_v2(result, 1, 2, 3).rstrip().endswith(',0')


def test_mode_and_counters_are_safe():
    assert ',SEARCH,1,' in format_target_v2({'valid': 1}, 1, 2, 3, 'INVALID')
    normalized = normalize_result_for_protocol({'valid': 1, 'angle_rad': math.inf})
    assert normalized['angle_rad'] == 0.0


def test_stdout_transport_one_line_and_bytes():
    stream = io.StringIO(); protocol = TargetProtocol('stdout', diagnostic_writer=stream)
    written = protocol.send_target(None, 4, 5, 6)
    assert written == len(stream.getvalue().encode())
    assert stream.getvalue().count('\n') == 1


class FakeUSB:
    def __init__(self, result=None, error=None): self.result, self.error, self.data = result, error, []
    def send(self, payload):
        if self.error: raise self.error
        self.data.append(payload)
        return self.result if self.result is not None else len(payload)


def test_usb_complete_send():
    usb = FakeUSB(); protocol = TargetProtocol('usb_vcp', usb=usb)
    assert protocol.send_target({'valid': 0}, 1, 2, 3) > 0 and len(usb.data) == 1


def test_usb_partial_zero_and_exception_counted_and_rate_limited():
    for result, error in ((1, None), (0, None), (None, RuntimeError('bad\nline'))):
        stream = io.StringIO(); usb = FakeUSB(result=result, error=error)
        protocol = TargetProtocol('usb_vcp', usb=usb, diagnostic_writer=stream)
        assert protocol.send_target(None, 1, 2, 3, now_ms=100) is False
        assert protocol.protocol_error_count == 1
        protocol.send_target(None, 2, 3, 4, now_ms=500)
        assert stream.getvalue().count('D_PROTOCOL_ERROR') == 1


def test_invalid_frame_never_reuses_old_coordinates():
    valid = {'valid': 1, 'cx': 123, 'cy': 88, 'outer_diameter_px': 50,
             'inner_diameter_px': 30, 'angle_rad': 1.0, 'confidence': 80}
    assert ',123,88,50,30,1.0000,80\n' in format_target_v2(valid, 1, 1, 1)
    assert ',0,0,0,0,0.0000,0\n' in format_target_v2({'valid': 0}, 2, 2, 2)
