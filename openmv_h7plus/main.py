"""OpenMV H7 Plus D-task detector entry (legacy detector adapter for now)."""

import pyb
import time

from camera_config import configure_camera
from detector import detect_red
from protocol import TargetProtocol


OUTPUT_PERIOD_MS = 50  # 约 20 Hz


def run():
    # CSI 只在启动时创建一次，循环中复用同一个摄像头对象。
    camera = configure_camera()
    protocol = TargetProtocol()
    clock = time.clock()
    last_output_ms = pyb.millis()

    while True:
        clock.tick()
        try:
            image = camera.snapshot()
            result = detect_red(image)
        except Exception:
            # 单帧错误不退出循环。标准无目标行对 ROS 2 桥接端最安全。
            result = None

        now_ms = pyb.millis()
        # pyb.elapsed_millis 可正确处理毫秒计数器回绕。
        if pyb.elapsed_millis(last_output_ms) >= OUTPUT_PERIOD_MS:
            try:
                protocol.send_target(result)
            except Exception:
                # USB 暂时未连接时继续处理后续帧，连接恢复后可继续输出。
                pass
            last_output_ms = now_ms


run()
