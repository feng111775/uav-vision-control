"""Normalize simulated car starts into idempotent mission events."""

import json
import math
import os
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .stage4c_core import StartGate, StartProtocol


def heartbeat_is_current(last_ns, now_ns, timeout_s):
    """Return true only for a finite, non-future heartbeat inside its window."""
    if last_ns is None:
        return False
    values = (float(last_ns), float(now_ns), float(timeout_s))
    if not all(math.isfinite(value) for value in values) or timeout_s <= 0.0:
        return False
    age_ns = now_ns - last_ns
    return 0 <= age_ns <= timeout_s * 1e9


class StartTransport:
    """Fail-closed transport configuration without opening any device."""

    SUPPORTED = {'simulation', 'serial', 'udp'}

    def __init__(self, kind, real_enabled=False, serial_device='',
                 serial_baud=0, udp_host='', udp_port=0):
        self.kind = str(kind)
        if self.kind not in self.SUPPORTED:
            raise ValueError('unsupported start transport')
        if self.kind == 'simulation':
            if real_enabled:
                raise ValueError('simulation transport cannot be real-enabled')
            return
        if not real_enabled:
            raise ValueError('real transport requires explicit enable')
        if self.kind == 'serial' and (
                not str(serial_device) or int(serial_baud) <= 0):
            raise ValueError('serial device and baud must be explicitly set')
        if self.kind == 'udp' and (
                not str(udp_host) or not 1 <= int(udp_port) <= 65535):
            raise ValueError('UDP host and port must be explicitly set')
        raise ValueError(
            'real transport adapter is not implemented; no device opened')


class CarStartGateway(Node):
    """Simulation gateway; it has no flight-control publisher."""

    def __init__(self):
        super().__init__('car_start_gateway')
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('transport', 'simulation')
        self.declare_parameter('real_transport_enabled', False)
        self.declare_parameter('serial_device', '')
        self.declare_parameter('serial_baud', 0)
        self.declare_parameter('udp_host', '')
        self.declare_parameter('udp_port', 0)
        self.declare_parameter('start_packet_max_age_s', 2.0)
        self.declare_parameter('heartbeat_timeout_s', 2.0)
        heartbeat_timeout = float(
            self.get_parameter('heartbeat_timeout_s').value)
        if not math.isfinite(heartbeat_timeout) or heartbeat_timeout <= 0.0:
            raise ValueError('heartbeat_timeout_s must be finite and positive')
        self.heartbeat_timeout_s = heartbeat_timeout
        self.transport = StartTransport(
            self.get_parameter('transport').value,
            self.get_parameter('real_transport_enabled').value,
            self.get_parameter('serial_device').value,
            self.get_parameter('serial_baud').value,
            self.get_parameter('udp_host').value,
            self.get_parameter('udp_port').value)
        self.declare_parameter(
            'start_state_path', os.path.join(
                tempfile.gettempdir(), 'uav_mission_start_gate.json'))
        self.gate = StartGate(
            str(self.get_parameter('start_state_path').value))
        self.manager_ready = False
        self.manager_busy = False
        self.last_heartbeat_ns = None
        self.publisher = self.create_publisher(
            String, '/uav_mission/events/start', 10)
        self.result_publisher = self.create_publisher(
            String, '/uav_mission/events/start_result', 10)
        self.transport_status_publisher = self.create_publisher(
            String, '/uav_mission/transport/status', 10)
        self.create_subscription(
            String, '/uav_mission/sim/car_start', self._start, 10)
        self.create_subscription(
            String, '/uav_mission/state', self._state, 10)
        self.create_subscription(
            String, '/uav_mission/readiness', self._readiness, 10)
        self.create_timer(0.2, self._transport_status)

    def _state(self, msg):
        try:
            state = str(json.loads(msg.data)['state'])
        except (KeyError, TypeError, json.JSONDecodeError):
            self.manager_ready = False
            return
        self.manager_busy = state != 'WAIT_FOR_START'
        if self.manager_busy:
            self.manager_ready = False

    def _readiness(self, msg):
        """Consume the manager's PX4-aware pre-flight readiness gate."""
        try:
            status = json.loads(msg.data)
            self.manager_ready = bool(status['ready'])
            self.manager_busy = bool(status['busy'])
        except (KeyError, TypeError, json.JSONDecodeError):
            self.manager_ready = False

    def _start(self, msg):
        if not self.get_parameter('simulation_mode').value:
            self.get_logger().warning('simulation start ignored outside simulation')
            return
        try:
            request = json.loads(msg.data)
            parsed = StartProtocol.parse(
                request, self.get_clock().now().nanoseconds,
                float(self.get_parameter('start_packet_max_age_s').value))
            session_id = str(parsed['session_id'])
            start_id = str(parsed['start_id'])
            counter = int(parsed['sender_counter'])
            command = str(parsed['command'])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning('malformed simulated start ignored')
            return
        now_ns = self.get_clock().now().nanoseconds
        if command == 'HEARTBEAT':
            self.last_heartbeat_ns = now_ns
            self._publish_result(
                'HEARTBEAT', session_id, start_id)
            return
        valid = command == 'CAR_START'
        heartbeat_current = (
            heartbeat_is_current(
                self.last_heartbeat_ns, now_ns, self.heartbeat_timeout_s))
        status = self.gate.accept_protocol(
            session_id, start_id, valid,
            self.manager_ready and heartbeat_current,
            self.manager_busy, counter)
        self._publish_result(status, session_id, start_id)
        if status != 'START_ACCEPTED':
            self.get_logger().warning('%s session=%s start=%s' % (
                status, session_id, start_id))
            return
        event = {
            'type': 'CAR_START',
            'protocol_version': StartProtocol.VERSION,
            'session_id': session_id,
            'start_id': start_id,
            'task_id': session_id + ':' + start_id,
            'stamp_ns': now_ns,
            'valid': True,
        }
        output = String()
        output.data = json.dumps(event, separators=(',', ':'))
        self.publisher.publish(output)
        self.get_logger().info(
            'START_ACCEPTED session=%s start=%s' % (session_id, start_id))

    def _publish_result(self, status, session_id, start_id):
        """Publish protocol status without opening or controlling hardware."""
        result = String()
        result.data = json.dumps({
            'type': status,
            'session_id': session_id,
            'start_id': start_id,
            'stamp_ns': self.get_clock().now().nanoseconds,
        }, separators=(',', ':'))
        self.result_publisher.publish(result)

    def _transport_status(self):
        """Publish heartbeat freshness without claiming a hardware link."""
        now_ns = self.get_clock().now().nanoseconds
        connected = heartbeat_is_current(
            self.last_heartbeat_ns, now_ns, self.heartbeat_timeout_s)
        output = String()
        output.data = json.dumps({
            'transport': self.transport.kind,
            'connected': connected,
            'stamp_ns': now_ns,
        }, separators=(',', ':'))
        self.transport_status_publisher.publish(output)


def main(args=None):
    """Run the car start gateway."""
    rclpy.init(args=args)
    node = CarStartGateway()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
