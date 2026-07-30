"""Receive validated D_TARGET detections from OpenMV H7 Plus."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import serial
from serial import SerialException
from std_msgs.msg import Float32MultiArray, Float64, String

from .d_task_schema import invalid_detection, validate_detection


def parse_detection_line(line, allow_legacy_protocol=False):
    """Parse D_TARGET; legacy mode only aliases TARGET to the new semantics."""
    fields = line.strip().split(',')
    if len(fields) != 8:
        raise ValueError('protocol requires 8 comma-separated fields')
    allowed = ('D_TARGET', 'TARGET') if allow_legacy_protocol else ('D_TARGET',)
    if fields[0] not in allowed:
        raise ValueError('unsupported protocol prefix')
    try:
        valid = int(fields[1])
        data = [float(valid)] + [float(value) for value in fields[2:]]
    except ValueError as error:
        raise ValueError('protocol contains a non-numeric value') from error
    if fields[0] == 'TARGET' and valid != 0:
        raise ValueError(
            'legacy TARGET valid detections are ambiguous and are not converted')
    return validate_detection(data)


def parse_status_line(line):
    fields = line.strip().split(',')
    if len(fields) != 2 or fields[0] != 'D_STATUS':
        raise ValueError('status protocol requires D_STATUS,state')
    if fields[1] not in ('TRACKING', 'LOST', 'CROSS_INVALID',
                         'DETECT_ERROR'):
        raise ValueError('invalid D_STATUS value')
    return fields[1]

def is_diagnostic_line(line):
    """Identify non-protocol performance diagnostics from OpenMV."""
    return line.startswith(('D_VISION,', 'D_TIMING,'))


class H7BridgeNode(Node):
    def __init__(self):
        super().__init__('h7_bridge_node')
        self.declare_parameter('port', '/dev/dtask_openmv')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('detection_topic', '/vision/h7/detection')
        self.declare_parameter('status_topic', '/vision/h7/status')
        self.declare_parameter('target_age_topic', '/vision/h7/target_age_ms')
        self.declare_parameter('data_timeout_sec', 0.5)
        self.declare_parameter('allow_legacy_protocol', False)
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.allow_legacy = self.get_parameter('allow_legacy_protocol').value
        self.data_timeout = self.get_parameter('data_timeout_sec').value
        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('detection_topic').value, 10)
        self.status_publisher = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.age_publisher = self.create_publisher(
            Float64, self.get_parameter('target_age_topic').value, 10)
        self.serial_port = None
        self._rx_buffer = bytearray()
        self._last_detection_time = None
        self._last_valid_time = None
        self._stale_published = False
        self._disconnected_published = False
        self.create_timer(0.01, self._read_serial)
        self.create_timer(2.0, self._open_serial)
        self.create_timer(0.1, self._check_timeout)
        self._open_serial()

    def _warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def _publish_status(self, status):
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def _publish_detection(self, data):
        message = Float32MultiArray()
        message.data = data
        self.publisher.publish(message)

    def _publish_invalid(self):
        self._publish_detection(invalid_detection())

    def _mark_disconnected(self):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except (SerialException, OSError):
                pass
        self.serial_port = None
        self._rx_buffer.clear()
        if not self._disconnected_published:
            self._publish_invalid()
            self._publish_status('DISCONNECTED')
            self._disconnected_published = True
        self._stale_published = True

    def _open_serial(self):
        if self.serial_port is not None and self.serial_port.is_open:
            return
        try:
            self.serial_port = serial.Serial(
                self.port, self.baudrate, timeout=0.005)
            self._rx_buffer.clear()
            self._disconnected_published = False
            self._stale_published = False
            self._last_detection_time = time.monotonic()
            self.get_logger().info('H7 serial connected: %s' % self.port)
        except (SerialException, OSError, ValueError) as error:
            self._mark_disconnected()
            self._warn('H7 serial unavailable: %s' % error)

    def _process_line(self, raw):
        try:
            line = raw.decode('ascii').strip()
            if not line:
                return
            if is_diagnostic_line(line):
                return
            if line.startswith('D_STATUS,'):
                self._publish_status(parse_status_line(line))
                return
            data = parse_detection_line(line, self.allow_legacy)
        except (UnicodeDecodeError, TypeError, ValueError) as error:
            self._warn('invalid H7 line ignored: %s' % error)
            return
        now = time.monotonic()
        self._last_detection_time = now
        self._stale_published = False
        if data[0] == 1.0:
            self._last_valid_time = now
        self._publish_detection(data)

    def _read_serial(self):
        if self.serial_port is None or not self.serial_port.is_open:
            return
        try:
            available = self.serial_port.in_waiting
            chunk = self.serial_port.read(available or 1)
            if chunk:
                self._rx_buffer.extend(chunk)
            while b'\n' in self._rx_buffer:
                raw, _, remainder = self._rx_buffer.partition(b'\n')
                self._rx_buffer = bytearray(remainder)
                self._process_line(raw)
            if len(self._rx_buffer) > 1024:
                self._rx_buffer.clear()
                self._warn('oversized partial H7 line discarded')
        except (SerialException, OSError) as error:
            self._warn('H7 serial read failed: %s' % error)
            self._mark_disconnected()

    def _check_timeout(self):
        now = time.monotonic()
        age = Float64()
        age.data = (
            (now - self._last_valid_time) * 1000.0
            if self._last_valid_time is not None else -1.0)
        self.age_publisher.publish(age)
        if self.serial_port is None or self._last_detection_time is None:
            return
        if now - self._last_detection_time > self.data_timeout:
            if not self._stale_published:
                self._publish_invalid()
                self._publish_status('STALE')
                self._stale_published = True

    def destroy_node(self):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = H7BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
