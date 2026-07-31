"""Read-only-safe UDP car START gateway; publishes only mission_start."""

import socket
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8

from .car_protocol import parse_frame


class CarStartGateway(Node):
    def __init__(self):
        super().__init__('car_start_gateway')
        self.declare_parameter('communication_only', True)
        self.declare_parameter('udp_bind_host', '127.0.0.1')
        self.declare_parameter('udp_bind_port', 4210)
        self.declare_parameter('allowed_source_ip', '127.0.0.1')
        self.declare_parameter('receive_timeout_seconds', 0.5)
        self.mission_pub = self.create_publisher(Bool, '/car/mission_start', 10)
        self.progress_pub = self.create_publisher(UInt8, '/car/progress', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.get_parameter('udp_bind_host').value,
                        int(self.get_parameter('udp_bind_port').value)))
        self.sock.setblocking(False)
        self.last_packet = time.monotonic()
        self.last_run = None
        self.started_runs = set()
        self.create_timer(0.01, self._poll)
        self.create_timer(0.1, self._link_check)

    def _poll(self):
        while True:
            try:
                data, address = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            if address[0] != self.get_parameter('allowed_source_ip').value:
                continue
            self.last_packet = time.monotonic()
            try:
                message = parse_frame(data.decode('ascii'))
            except (UnicodeDecodeError, ValueError):
                continue
            if message['kind'] == 'CAR':
                self.last_run = message['run_id']
                output = UInt8(data=min(100, message['progress_permille'] // 10))
                self.progress_pub.publish(output)
                continue
            if message['event'] != 'START' or message['run_id'] in self.started_runs:
                continue
            self.started_runs.add(message['run_id'])
            if self.get_parameter('communication_only').value:
                self.get_logger().warning(
                    'START validated but mission trigger suppressed by communication_only')
                continue
            self.mission_pub.publish(Bool(data=True))

    def _link_check(self):
        if time.monotonic() - self.last_packet > float(
                self.get_parameter('receive_timeout_seconds').value):
            return

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CarStartGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
