"""D_TARGET serialization and OpenMV USB CDC transport."""

try:
    from pyb import USB_VCP
except ImportError:  # Allows PC-side serialization regression tests.
    USB_VCP = None


NO_TARGET_LINE = "D_TARGET,0,0,0,0,0,0,0\n"


def format_target_line(result):
    if result is None or not result.get("valid", 0):
        return NO_TARGET_LINE
    return "D_TARGET,%d,%d,%d,%d,%d,%.4f,%d\n" % (
        1, result["cx"], result["cy"],
        result["outer_diameter_px"], result["inner_diameter_px"],
        result["angle_rad"], max(0, min(100, result["confidence"])))


class TargetProtocol:
    """Send one complete ASCII event through the USB VCP."""

    def __init__(self, usb=None):
        if usb is None:
            if USB_VCP is None:
                raise RuntimeError("USB_VCP is only available on OpenMV")
            usb = USB_VCP()
        self._usb = usb

    def send_target(self, result):
        self._usb.send(format_target_line(result).encode("ascii"))

    def send_no_target(self):
        self._usb.send(NO_TARGET_LINE.encode("ascii"))

    def send_status(self, status):
        safe = status if status in (
            "TRACKING", "LOST", "CROSS_INVALID", "DETECT_ERROR") else "LOST"
        self._usb.send(("D_STATUS,%s\n" % safe).encode("ascii"))
