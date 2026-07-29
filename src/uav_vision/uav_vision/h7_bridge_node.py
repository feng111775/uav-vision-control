"""Receive validated D_TARGET detections from OpenMV H7 Plus."""

import rclpy
from rclpy.node import Node
import serial
from serial import SerialException
from std_msgs.msg import Float32MultiArray, String

from .d_task_schema import validate_detection


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


class H7BridgeNode(Node):
    def __init__(self):
        super().__init__('h7_bridge_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('detection_topic', '/vision/h7/detection')
        self.declare_parameter('status_topic', '/vision/h7/status')
        self.declare_parameter('allow_legacy_protocol', False)
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.allow_legacy = self.get_parameter('allow_legacy_protocol').value
        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('detection_topic').value, 10)
        self.status_publisher = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.serial_port = None
        self.create_timer(0.01, self._read_serial)
        self.create_timer(2.0, self._open_serial)
        self._open_serial()

    def _warn(self, message):
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def _open_serial(self):
        if self.serial_port is not None and self.serial_port.is_open:
            return
        try:
            self.serial_port = serial.Serial(
                self.port, self.baudrate, timeout=0.005)
            self.get_logger().info('H7 serial connected: %s' % self.port)
        except (SerialException, OSError, ValueError) as error:
            self.serial_port = None
            self._warn('H7 serial unavailable: %s' % error)

    def _read_serial(self):
        if self.serial_port is None or not self.serial_port.is_open:
            return
        try:
            for _ in range(100):
                raw = self.serial_port.readline()
                if not raw:
                    break
                try:
                    line = raw.decode('ascii').strip()
                    if line.startswith('D_STATUS,'):
                        status = parse_status_line(line)
                        status_message = String()
                        status_message.data = status
                        self.status_publisher.publish(status_message)
                        continue
                    data = parse_detection_line(line, self.allow_legacy)
                except (UnicodeDecodeError, TypeError, ValueError) as error:
                    self._warn('invalid H7 line ignored: %s' % error)
                    continue
                message = Float32MultiArray()
                message.data = data
                self.publisher.publish(message)
        except (SerialException, OSError) as error:
            self._warn('H7 serial read failed: %s' % error)
            try:
                self.serial_port.close()
            except (SerialException, OSError):
                pass
            self.serial_port = None

    def destroy_node(self):
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = H7BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
