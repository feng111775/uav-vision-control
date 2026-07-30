"""OpenMV H7 Plus formal D-task detection and USB CDC heartbeat loop."""

import pyb
import time

from camera_config import configure_camera
from detector import DTaskDetector
from protocol import TargetProtocol


OUTPUT_PERIOD_MS = 50
LOG_PERIOD_MS = 2000
TIMING_ENABLED = False
TIMING_LOG_PERIOD_MS = 10000


def run():
    camera = configure_camera()
    detector = DTaskDetector(enable_timing=TIMING_ENABLED)
    protocol = TargetProtocol()
    clock = time.clock()
    last_output_ms = pyb.millis()
    last_log_ms = last_output_ms
    last_status = "STARTING"
    last_timing_ms = last_output_ms

    while True:
        frame_started = detector.timing.begin()
        clock.tick()
        try:
            snapshot_started = detector.timing.begin()
            image = camera.snapshot()
            detector.timing.end("snapshot", snapshot_started)
            result = detector.detect(image)
            last_status = result.get("status", "LOST")
        except Exception as error:
            result = None
            last_status = "DETECT_ERROR"

        now_ms = pyb.millis()
        if pyb.elapsed_millis(last_output_ms) >= OUTPUT_PERIOD_MS:
            protocol_started = detector.timing.begin()
            try:
                protocol.send_target(result)
                protocol.send_status(last_status)
            except Exception:
                # USB disconnect never stops camera/detector processing.
                pass
            detector.timing.end("protocol_send", protocol_started)
            last_output_ms = now_ms
        if pyb.elapsed_millis(last_log_ms) >= LOG_PERIOD_MS:
            try:
                print("D_VISION,status=%s,fps=%.1f" % (
                    last_status, clock.fps()))
            except Exception:
                pass
            last_log_ms = now_ms
        if TIMING_ENABLED and (
                pyb.elapsed_millis(last_timing_ms) >= TIMING_LOG_PERIOD_MS):
            for name, values in detector.timing.summary().items():
                print("D_TIMING,%s,avg_ms=%.3f,p95_ms=%.3f,max_ms=%.3f,n=%d" % (
                    name, values[0], values[1], values[2], values[3]))
            last_timing_ms = now_ms
        detector.timing.end("frame_total", frame_started)


run()
