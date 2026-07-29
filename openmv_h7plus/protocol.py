"""ROS 2 H7 桥接节点兼容的文本协议。"""

from pyb import USB_VCP


NO_TARGET_LINE = "D_TARGET,0,0,0,0,0,0,0\n"


class TargetProtocol:
    """通过 OpenMV 的 USB CDC 虚拟串口发送完整文本行。"""

    def __init__(self):
        # OpenMV IDE 5.0 官方 USB VCP 示例使用 USB_VCP() 和 send()。
        self._usb = USB_VCP()

    def send_target(self, result):
        if result is None:
            self.send_no_target()
            return
        line = "D_TARGET,%d,%d,%d,%d,%d,%.4f,%d\n" % (
            result["valid"],
            result["cx"],
            result["cy"],
            result["outer_diameter_px"],
            result["inner_diameter_px"],
            result["angle_rad"],
            result["confidence"],
        )
        self._usb.send(line.encode("ascii"))

    def send_no_target(self):
        self._usb.send(NO_TARGET_LINE.encode("ascii"))
