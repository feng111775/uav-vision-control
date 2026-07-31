"""OpenMV H7 Plus detection loop with observable V2 protocol failures."""
import pyb
import time

from camera_config import configure_camera
from detector_legacy import LegacyDetector
from detector_fast import FastV2Detector
from protocol import TargetProtocol
from config import (DETECTOR_BACKEND, OUTPUT_PERIOD_MS, TIMING_ENABLED,
                    PROTOCOL_TRANSPORT, USB_VCP_ID)

DTaskDetector = LegacyDetector
LOG_PERIOD_MS = 2000
STATUS_PERIOD_MS = 1000
TIMING_LOG_PERIOD_MS = 10000


def _error_line(writer, count, error, now_ms, last_ms):
    if pyb.elapsed_millis(last_ms) < 1000:
        return last_ms
    message = str(error).replace('\n', ' ')[:120]
    try:
        writer.write('D_VISION_ERROR,count=%d,type=%s,message=%s\n' %
                     (count, type(error).__name__, message))
    except Exception:
        pass
    return now_ms


def run():
    camera = configure_camera()
    detector = (LegacyDetector if DETECTOR_BACKEND == 'legacy' else FastV2Detector)(
        enable_timing=TIMING_ENABLED)
    protocol = TargetProtocol(transport=PROTOCOL_TRANSPORT,
                              usb_vcp_id=USB_VCP_ID)
    protocol.send_boot()
    clock = time.clock()
    last_output_ms = pyb.millis(); frame_sequence = 0
    last_log_ms = last_output_ms; last_status_ms = last_output_ms
    last_timing_ms = last_output_ms; last_error_ms = last_output_ms - 1000
    last_status = 'LOST'; capture_error_count = 0; detect_error_count = 0

    while True:
        frame_started_us = pyb.micros()
        timing_started = detector.timing.begin()
        clock.tick()
        image = None; capture_ms = pyb.millis(); result = None
        camera_ok = False; algorithm_ok = False
        try:
            image = camera.snapshot(); capture_ms = pyb.millis()
            frame_sequence += 1; camera_ok = True
            detector.timing.end('snapshot', timing_started)
        except Exception as error:
            capture_error_count += 1
            last_error_ms = _error_line(protocol._diagnostic_writer,
                                        capture_error_count, error,
                                        pyb.millis(), last_error_ms)
        if image is not None:
            try:
                result = detector.detect(image)
                result = result if isinstance(result, dict) else None
                last_status = (result or {}).get('status', 'LOST')
                algorithm_ok = result is not None
            except Exception as error:
                detect_error_count += 1; result = None
                last_status = 'DETECT_ERROR'
                last_error_ms = _error_line(protocol._diagnostic_writer,
                                            detect_error_count, error,
                                            pyb.millis(), last_error_ms)
        now_ms = pyb.millis()
        if frame_sequence and (OUTPUT_PERIOD_MS == 0 or
                               pyb.elapsed_millis(last_output_ms) >= OUTPUT_PERIOD_MS):
            protocol_started = detector.timing.begin()
            processing_us = max(0, pyb.elapsed_micros(frame_started_us))
            protocol.send_target(result, frame_sequence, capture_ms,
                                 processing_us, getattr(detector, 'mode', 'SEARCH'),
                                 now_ms)
            detector.timing.end('protocol_send', protocol_started)
            last_output_ms = now_ms
        if pyb.elapsed_millis(last_status_ms) >= STATUS_PERIOD_MS:
            protocol.send_status_v2(frame_sequence,
                                    getattr(detector, 'mode', 'SEARCH'),
                                    camera_ok, algorithm_ok, clock.fps(), now_ms)
            last_status_ms = now_ms
        if pyb.elapsed_millis(last_log_ms) >= LOG_PERIOD_MS:
            try:
                protocol._diagnostic_writer.write(
                    'D_VISION,status=%s,fps=%.1f\n' %
                    (last_status, clock.fps()))
            except Exception:
                pass
            last_log_ms = now_ms
        if TIMING_ENABLED and pyb.elapsed_millis(last_timing_ms) >= TIMING_LOG_PERIOD_MS:
            for name, values in detector.timing.summary().items():
                protocol._diagnostic_writer.write(
                    'D_TIMING,%s,avg_ms=%.3f,p50_ms=%.3f,p95_ms=%.3f,max_ms=%.3f,n=%d\n' %
                    (name, values[0], values[1], values[2], values[3], values[4]))
            last_timing_ms = now_ms
        detector.timing.end('frame_total', timing_started)


run()
