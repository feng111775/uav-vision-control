"""OpenMV H7 Plus formal D-task detection and USB CDC heartbeat loop."""

import pyb
import time

from camera_config import configure_camera
from detector import DTaskDetector
from protocol import TargetProtocol


OUTPUT_PERIOD_MS = 50
LOG_PERIOD_MS = 2000


def run():
    camera = configure_camera()
    detector = DTaskDetector()
    protocol = TargetProtocol()
    clock = time.clock()
    last_output_ms = pyb.millis()
    last_log_ms = last_output_ms
    last_status = "STARTING"

    while True:
        clock.tick()
        try:
            image = camera.snapshot()
            result = detector.detect(image)
            last_status = result.get("status", "LOST")
        except Exception as error:
            result = None
            last_status = "DETECT_ERROR"

        now_ms = pyb.millis()
        if pyb.elapsed_millis(last_output_ms) >= OUTPUT_PERIOD_MS:
            try:
                protocol.send_target(result)
                protocol.send_status(last_status)
            except Exception:
                # USB disconnect never stops camera/detector processing.
                pass
            last_output_ms = now_ms
        if pyb.elapsed_millis(last_log_ms) >= LOG_PERIOD_MS:
            try:
                print("D_VISION,status=%s,fps=%.1f" % (
                    last_status, clock.fps()))
            except Exception:
                pass
            last_log_ms = now_ms


run()
