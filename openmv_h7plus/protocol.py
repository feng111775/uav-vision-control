"""Robust D-task V2 formatting with selectable stdout/USB-VCP transport."""
import math
import sys

try:
    from pyb import USB_VCP
except ImportError:  # PC tests
    USB_VCP = None

try:
    from config import PROTOCOL_TRANSPORT, USB_VCP_ID, format_runtime_config_line
except ImportError:
    PROTOCOL_TRANSPORT, USB_VCP_ID = 'stdout', 0
    format_runtime_config_line = None


MODES = ('MISSION_IDLE', 'SEARCH', 'ACQUIRE', 'FOLLOW', 'DROP_ALIGN')
NO_TARGET_LINE = "D_TARGET,0,0,0,0,0,0,0\n"


def safe_int(value, default=0, minimum=None, maximum=None):
    try:
        if isinstance(value, bool):
            number = int(value)
        else:
            number = int(float(value))
    except (TypeError, ValueError, OverflowError):
        number = int(default)
    if minimum is not None and number < minimum:
        number = minimum
    if maximum is not None and number > maximum:
        number = maximum
    return number


def safe_float(value, default=0.0, minimum=None, maximum=None):
    try:
        number = float(value)
        if not math.isfinite(number):
            raise ValueError
    except (TypeError, ValueError, OverflowError):
        number = float(default)
    if minimum is not None and number < minimum:
        number = minimum
    if maximum is not None and number > maximum:
        number = maximum
    return number


def clamp_confidence(value):
    return safe_int(value, 0, 0, 100)


def _mode(value):
    value = str(value) if value is not None else ''
    return value if value in MODES else 'SEARCH'


def normalize_result_for_protocol(result):
    """Normalize detector dictionaries without retaining stale geometry."""
    source = result if isinstance(result, dict) else {}
    valid = bool(safe_int(source.get('valid', 0), 0, 0, 1))
    normalized = {'valid': int(valid)}
    if not valid:
        normalized.update({'cx': 0, 'cy': 0, 'outer_diameter_px': 0,
                           'inner_diameter_px': 0, 'angle_rad': 0.0,
                           'confidence': 0})
        return normalized
    normalized.update({
        'cx': safe_int(source.get('cx'), 0, 0),
        'cy': safe_int(source.get('cy'), 0, 0),
        'outer_diameter_px': safe_int(source.get('outer_diameter_px'), 0, 0),
        'inner_diameter_px': safe_int(source.get('inner_diameter_px'), 0, 0),
        'angle_rad': safe_float(source.get('angle_rad'), 0.0),
        'confidence': clamp_confidence(source.get('confidence')),
    })
    return normalized


def format_target_line(result):
    normalized = normalize_result_for_protocol(result)
    if not normalized['valid']:
        return NO_TARGET_LINE
    return "D_TARGET,%d,%d,%d,%d,%d,%.4f,%d\n" % (
        1, normalized['cx'], normalized['cy'],
        normalized['outer_diameter_px'], normalized['inner_diameter_px'],
        normalized['angle_rad'], normalized['confidence'])


def format_target_v2(result, frame_sequence, capture_ticks_ms, processing_us,
                     mode='SEARCH'):
    normalized = normalize_result_for_protocol(result)
    return 'D_TARGET_V2,%d,%d,%d,%s,%d,%d,%d,%d,%d,%.4f,%d\n' % (
        safe_int(frame_sequence, 0, 0), safe_int(capture_ticks_ms, 0, 0),
        safe_int(processing_us, 0, 0), _mode(mode), normalized['valid'],
        normalized['cx'], normalized['cy'], normalized['outer_diameter_px'],
        normalized['inner_diameter_px'], normalized['angle_rad'],
        normalized['confidence'])


class TargetProtocol:
    """Send exactly one selected transport copy and expose error counters."""
    def __init__(self, transport=None, usb=None, usb_vcp_id=None,
                 diagnostic_writer=None):
        self.transport = transport or PROTOCOL_TRANSPORT
        self.usb_vcp_id = USB_VCP_ID if usb_vcp_id is None else usb_vcp_id
        self._usb = usb
        self.protocol_error_count = 0
        self._last_error_ms = -1000000
        self._diagnostic_writer = diagnostic_writer or sys.stdout
        if self.transport not in ('stdout', 'usb_vcp'):
            self.transport = 'stdout'
        if self.transport == 'usb_vcp' and self._usb is None:
            if USB_VCP is None:
                raise RuntimeError('USB_VCP is only available on OpenMV')
            self._usb = USB_VCP(self.usb_vcp_id)

    def _write_diagnostic(self, now_ms, error):
        if now_ms is None or now_ms - self._last_error_ms < 1000:
            return
        self._last_error_ms = now_ms
        message = str(error).replace('\n', ' ')[:120]
        try:
            self._diagnostic_writer.write(
                'D_PROTOCOL_ERROR,count=%d,type=%s,message=%s\n' %
                (self.protocol_error_count, type(error).__name__, message))
        except Exception:
            pass

    def _send_payload(self, payload, now_ms=None):
        data = payload.encode('ascii') if isinstance(payload, str) else payload
        try:
            if self.transport == 'stdout':
                written = self._diagnostic_writer.write(data.decode('ascii'))
                written = len(data) if written is None else int(written)
            else:
                written = self._usb.send(data)
                written = 0 if written is None else int(written)
            if written != len(data):
                raise IOError('partial transport write %d/%d' %
                              (written, len(data)))
            return written
        except Exception as error:
            self.protocol_error_count += 1
            self._write_diagnostic(now_ms, error)
            return False

    def send_target(self, result, frame_sequence=None, capture_ticks_ms=None,
                    processing_us=0, mode='SEARCH', now_ms=None):
        if frame_sequence is None:
            payload = format_target_line(result)
        else:
            payload = format_target_v2(result, frame_sequence,
                                       capture_ticks_ms, processing_us, mode)
        return self._send_payload(payload, now_ms)

    def send_no_target(self, now_ms=None):
        return self._send_payload(NO_TARGET_LINE, now_ms)

    def send_status(self, status, now_ms=None):
        safe = status if status in ('TRACKING', 'LOST', 'CROSS_INVALID',
                                    'DETECT_ERROR') else 'LOST'
        return self._send_payload('D_STATUS,%s\n' % safe, now_ms)

    def send_boot(self):
        return self._send_payload('D_BOOT_V2,transport=%s,vcp_id=%d,backend=%s\n' %
                                  (self.transport, self.usb_vcp_id,
                                   _config_backend()), None)

    def send_config(self):
        if format_runtime_config_line is None:
            return False
        return self._send_payload(format_runtime_config_line(), None)

    def send_status_v2(self, frame_sequence, mode, camera_ok, algorithm_ok,
                       fps, now_ms=None):
        return self._send_payload(
            'D_STATUS_V2,%d,%s,%d,%d,%d,%d,%.1f\n' %
            (safe_int(frame_sequence, 0, 0), _mode(mode), int(bool(camera_ok)),
             int(bool(algorithm_ok)), int(self.protocol_error_count == 0),
             self.protocol_error_count, safe_float(fps, 0.0, 0.0)), now_ms)


def _config_backend():
    try:
        from config import DETECTOR_BACKEND
        return DETECTOR_BACKEND
    except Exception:
        return 'unknown'
