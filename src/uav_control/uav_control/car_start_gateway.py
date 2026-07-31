"""Normalize car starts into idempotent mission events and ROS signals."""

import ipaddress
import json
import math
import os
import socket
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8

from .stage4c_core import StartGate, StartProtocol
from .udp_protocol import parse_stm32_frame, ParsedFrame, StartJournal


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
        self.real_enabled = bool(real_enabled)
        self.serial_device = str(serial_device)
        self.serial_baud = int(serial_baud)
        self.udp_host = str(udp_host)
        self.udp_port = int(udp_port)
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
        if self.kind == 'serial':
            raise ValueError(
                'real transport adapter is not implemented; no device opened')


def parse_start_text_frame(payload: str):
    """Accept a minimal compatible text start frame without widening scope."""
    text = str(payload).strip()
    if not text:
        raise ValueError('empty frame')
    if text == 'CAR_START':
        return {'command': 'CAR_START', 'session_id': 'udp-text',
                'start_id': 'default', 'sender_counter': 0}
    if text.startswith('START,'):
        _, sequence = text.split(',', 1)
        sequence = sequence.strip()
        if not sequence:
            raise ValueError('empty sequence')
        return {'command': 'CAR_START', 'session_id': 'udp-text',
                'start_id': sequence, 'sender_counter': int(sequence)}
    if text.startswith('CAR_START,'):
        _, sequence = text.split(',', 1)
        sequence = sequence.strip()
        if not sequence:
            raise ValueError('empty sequence')
        return {'command': 'CAR_START', 'session_id': 'udp-text',
                'start_id': sequence, 'sender_counter': int(sequence)}
    raise ValueError('unknown text frame')


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
        self.declare_parameter('udp_poll_period_s', 0.05)
        self.declare_parameter('udp_peer_host', '')
        self.declare_parameter('udp_peer_port', 0)
        self.declare_parameter('udp_require_peer', False)
        self.declare_parameter('max_datagram_bytes', 512)
        self.declare_parameter('max_packets_per_tick', 32)
        self.declare_parameter('running_fallback_count', 3)
        self.declare_parameter('running_fallback_max_gap_s', 0.5)
        self.declare_parameter('car_point_b_progress_permille', -1)
        self.declare_parameter('car_point_c_progress_permille', -1)
        self.declare_parameter('car_point_d_progress_permille', -1)
        self.declare_parameter('car_return_progress_permille', -1)
        self.declare_parameter('trigger_repeat_count', 3)
        self.declare_parameter('trigger_repeat_period_s', 0.1)
        self.declare_parameter('subscriber_wait_timeout_s', 5.0)
        self.declare_parameter('udp_journal_path', os.path.join(
            tempfile.gettempdir(), 'uav_car_udp_journal.json'))
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
        self.udp_peer_host = str(self.get_parameter('udp_peer_host').value)
        self.udp_peer_port = int(self.get_parameter('udp_peer_port').value)
        self.udp_require_peer = bool(
            self.get_parameter('udp_require_peer').value)
        self.max_datagram_bytes = int(
            self.get_parameter('max_datagram_bytes').value)
        self.max_packets_per_tick = int(
            self.get_parameter('max_packets_per_tick').value)
        self.running_fallback_count = int(
            self.get_parameter('running_fallback_count').value)
        self.running_fallback_max_gap_s = float(
            self.get_parameter('running_fallback_max_gap_s').value)
        threshold_names = (
            'car_point_b_progress_permille',
            'car_point_c_progress_permille',
            'car_point_d_progress_permille',
            'car_return_progress_permille')
        self.progress_thresholds = tuple(
            int(self.get_parameter(name).value) for name in threshold_names)
        self.trigger_repeat_count = int(
            self.get_parameter('trigger_repeat_count').value)
        self.trigger_repeat_period_s = float(
            self.get_parameter('trigger_repeat_period_s').value)
        self.subscriber_wait_timeout_s = float(
            self.get_parameter('subscriber_wait_timeout_s').value)
        udp_journal_path = str(
            self.get_parameter('udp_journal_path').value).strip()
        if not udp_journal_path:
            raise ValueError('udp_journal_path must not be empty')
        if self.transport.kind == 'udp' and not self.transport.udp_host:
            raise ValueError('udp_host/listen_address must not be empty')
        if self.transport.kind == 'udp':
            try:
                ipaddress.ip_address(self.transport.udp_host)
            except ValueError as error:
                raise ValueError('udp_host/listen_address must be an IP') from error
        if self.udp_peer_host:
            try:
                ipaddress.ip_address(self.udp_peer_host)
            except ValueError as error:
                raise ValueError('udp_peer_host must be an IP') from error
        if (self.udp_require_peer and
                (not self.udp_peer_host or not 1 <= self.udp_peer_port <= 65535)):
            raise ValueError('UDP peer host and port are required')
        if not self.udp_peer_host and self.udp_peer_port:
            raise ValueError('UDP peer host is required with peer port')
        if not 64 <= self.max_datagram_bytes <= 65535:
            raise ValueError('max_datagram_bytes is outside safe range')
        if self.max_packets_per_tick < 1:
            raise ValueError('max_packets_per_tick must be positive')
        if (not math.isfinite(self.running_fallback_max_gap_s) or
                self.running_fallback_max_gap_s <= 0.0):
            raise ValueError('running_fallback_max_gap_s must be positive')
        if self.running_fallback_count < 1:
            raise ValueError('running_fallback_count must be positive')
        configured = [value for value in self.progress_thresholds if value >= 0]
        if (any(value > 1000 for value in configured) or
                configured != sorted(configured) or
                (configured and configured[0] == 0)):
            raise ValueError(
                'car progress thresholds must be ordered permille values in 1..1000')
        if self.trigger_repeat_count < 1 or not math.isfinite(
                self.trigger_repeat_period_s) or self.trigger_repeat_period_s <= 0:
            raise ValueError('trigger repeat parameters are invalid')
        if (not math.isfinite(self.subscriber_wait_timeout_s) or
                self.subscriber_wait_timeout_s <= 0.0):
            raise ValueError('subscriber_wait_timeout_s must be positive')
        self.udp_journal = StartJournal(
            self.get_parameter('udp_journal_path').value)
        self.declare_parameter(
            'start_state_path', os.path.join(
                tempfile.gettempdir(), 'uav_mission_start_gate.json'))
        self.gate = StartGate(
            str(self.get_parameter('start_state_path').value))
        self.manager_ready = False
        self.manager_busy = False
        self.last_heartbeat_ns = None
        self.last_frame_source = ''
        self.last_frame_sequence = None
        self.last_frame_stamp_ns = None
        self.last_valid_udp_monotonic = None
        self.last_link_state = None
        self.last_ping_monotonic = 0.0
        self.running_candidate_run_id = None
        self.running_candidate_count = 0
        self.running_candidate_last_monotonic = None
        self.udp_pending_run_id = self.udp_journal.pending_run_id
        self.udp_pending_source = self.udp_journal.pending_source
        self.udp_pending_received_monotonic = (
            time.monotonic() if self.udp_pending_run_id is not None else None)
        self.udp_request_received_monotonic = (
            self.udp_pending_received_monotonic)
        self.udp_publish_count = 0
        self.udp_first_publish_monotonic = None
        self.udp_last_publish_monotonic = None
        self.udp_socket = None
        self.publisher = self.create_publisher(
            String, '/uav_mission/events/start', 10)
        self.mission_start_publisher = self.create_publisher(
            Bool, '/car/mission_start', 10)
        self.result_publisher = self.create_publisher(
            String, '/uav_mission/events/start_result', 10)
        self.transport_status_publisher = self.create_publisher(
            String, '/uav_mission/transport/status', 10)
        self.progress_publisher = self.create_publisher(
            UInt8, '/car/progress', 10)
        self.telemetry_publisher = self.create_publisher(
            String, '/car/telemetry', 10)
        self.link_publisher = self.create_publisher(
            Bool, '/car/link_alive', 10)
        self.create_subscription(
            String, '/uav_mission/sim/car_start', self._start, 10)
        self.create_subscription(
            String, '/uav_mission/state', self._state, 10)
        self.create_subscription(
            String, '/uav_mission/readiness', self._readiness, 10)
        self.create_timer(0.2, self._transport_status)
        self.create_timer(0.02, self._deliver_udp_pending)
        self._setup_udp_transport()

    def destroy_node(self):
        if self.udp_socket is not None:
            try:
                self.udp_socket.close()
            except OSError:
                pass
            self.udp_socket = None
        return super().destroy_node()

    def _setup_udp_transport(self):
        if self.transport.kind != 'udp':
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.transport.udp_host, self.transport.udp_port))
        sock.setblocking(False)
        self.udp_socket = sock
        period = float(self.get_parameter('udp_poll_period_s').value)
        if (not math.isfinite(period)) or period <= 0.0:
            raise ValueError('udp_poll_period_s must be finite and positive')
        self.create_timer(period, self._poll_udp)
        self.get_logger().info(
            'UDP start listener bound to %s:%d' % (
                self.transport.udp_host, self.transport.udp_port))

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
        mission_start = Bool()
        mission_start.data = True
        self.mission_start_publisher.publish(mission_start)
        self.get_logger().info(
            'START_ACCEPTED session=%s start=%s' % (session_id, start_id))

    def _poll_udp(self):
        if self.udp_socket is None:
            return
        for _ in range(self.max_packets_per_tick):
            try:
                payload, address = self.udp_socket.recvfrom(
                    self.max_datagram_bytes + 1)
            except BlockingIOError:
                return
            except OSError as error:
                self.get_logger().error('UDP receive failed: %s' % error)
                return
            if len(payload) > self.max_datagram_bytes:
                self.get_logger().warning('ignored oversized UDP datagram')
                continue
            if (self.udp_require_peer and
                    (address[0] != self.udp_peer_host or
                     address[1] != self.udp_peer_port)):
                self.get_logger().warning(
                    'ignored UDP datagram from unexpected peer %s:%d' % address)
                continue
            self._handle_udp_frame(payload, address)

    def _handle_udp_frame(self, payload, address):
        source = '%s:%d' % address
        now_ns = self.get_clock().now().nanoseconds
        try:
            text = payload.decode('ascii')
        except UnicodeDecodeError:
            self.get_logger().warning('invalid UDP frame encoding from %s' % source)
            return
        if not text:
            self.get_logger().warning('empty UDP frame from %s' % source)
            return
        if text == 'PONG':
            self._mark_udp_alive()
            return
        if text.startswith('$'):
            try:
                frame = parse_stm32_frame(text, self.max_datagram_bytes)
            except ValueError as error:
                self.get_logger().warning('ignored invalid STM32 frame: %s' % error)
                return
            self._mark_udp_alive()
            self.telemetry_publisher.publish(String(data=frame.raw))
            self._handle_stm32_frame(frame)
            return
        try:
            request = json.loads(text)
        except json.JSONDecodeError:
            request = None
        try:
            if request is not None:
                parsed = StartProtocol.parse(
                    request, now_ns,
                    float(self.get_parameter('start_packet_max_age_s').value))
                session_id = str(parsed['session_id'])
                start_id = str(parsed['start_id'])
                counter = int(parsed['sender_counter'])
                command = str(parsed['command'])
            else:
                parsed = parse_start_text_frame(text)
                session_id = str(parsed['session_id'])
                start_id = str(parsed['start_id'])
                counter = int(parsed['sender_counter'])
                command = str(parsed['command'])
        except (TypeError, ValueError, KeyError) as error:
            self.get_logger().warning('ignored UDP frame from %s: %s' % (source, error))
            return

        self.last_frame_source = source
        self.last_frame_sequence = counter
        self.last_frame_stamp_ns = now_ns

        if command == 'HEARTBEAT':
            self.last_heartbeat_ns = now_ns
            self._mark_udp_alive()
            self._publish_result('HEARTBEAT', session_id, start_id)
            return

        status = self.gate.accept_protocol(
            session_id, start_id, command == 'CAR_START',
            self.manager_ready, self.manager_busy, counter)
        self._publish_result(status, session_id, start_id)
        self._mark_udp_alive()
        self.get_logger().info(
            'UDP %s from %s seq=%s session=%s start=%s' % (
                status, source, counter, session_id, start_id))
        if status != 'START_ACCEPTED':
            return

        event = String()
        event.data = json.dumps({
            'type': 'CAR_START',
            'protocol_version': StartProtocol.VERSION,
            'session_id': session_id,
            'start_id': start_id,
            'task_id': session_id + ':' + start_id,
            'stamp_ns': now_ns,
            'valid': True,
            'source': source,
            'sender_counter': counter,
        }, separators=(',', ':'))
        self.publisher.publish(event)
        mission_start = Bool()
        mission_start.data = True
        self.mission_start_publisher.publish(mission_start)

    def _mark_udp_alive(self):
        self.last_valid_udp_monotonic = time.monotonic()
        self._publish_link(True)

    def _handle_stm32_frame(self, frame: ParsedFrame):
        """Handle one validated ASCII frame without changing mission logic."""
        if frame.kind == 'EVT':
            event = frame.fields[2].upper()
            if event == 'START':
                self._queue_udp_start(frame.run_id, 'EVT_START')
            elif event in {'FINISH', 'TIMEOUT', 'ESTOP', 'REMOTE_STOP'}:
                self.running_candidate_run_id = None
                self.running_candidate_count = 0
            return
        state = int(frame.fields[2])
        progress = int(frame.fields[4])
        self.progress_publisher.publish(
            UInt8(data=self._progress_stage(progress)))
        if state != 1:
            self.running_candidate_run_id = None
            self.running_candidate_count = 0
            return
        now = time.monotonic()
        if (self.running_candidate_run_id == frame.run_id and
                self.running_candidate_last_monotonic is not None and
                now - self.running_candidate_last_monotonic <=
                self.running_fallback_max_gap_s):
            self.running_candidate_count += 1
        else:
            self.running_candidate_run_id = frame.run_id
            self.running_candidate_count = 1
        self.running_candidate_last_monotonic = now
        if self.running_candidate_count >= self.running_fallback_count:
            self._queue_udp_start(frame.run_id, 'CAR_RUNNING_FALLBACK')

    def _progress_stage(self, progress_permille):
        """
        Map calibrated wire progress to the frozen stage enum.

        A raw permille value is not compatible with `/car/progress`, whose
        contract is `0..5` stage values.  Uncalibrated thresholds therefore
        expose only the safe A/start stage and cannot claim B or D timing.
        """
        if progress_permille <= 0:
            return 0
        b, c, d, returned = self.progress_thresholds
        if returned >= 0 and progress_permille >= returned:
            return 5
        if d >= 0 and progress_permille >= d:
            return 4
        if c >= 0 and progress_permille >= c:
            return 3
        if b >= 0 and progress_permille >= b:
            return 2
        return 1

    def _queue_udp_start(self, run_id, source):
        status = self.udp_journal.begin(run_id, source)
        if status == 'ALREADY_PROCESSED':
            return
        if status == 'BUSY':
            return
        if status == 'LOCKED':
            self.get_logger().error('UDP start journal is locked')
            return
        self.udp_pending_run_id = int(run_id)
        self.udp_pending_source = str(source)
        self.udp_pending_received_monotonic = time.monotonic()
        self.udp_request_received_monotonic = self.udp_pending_received_monotonic
        self.udp_publish_count = 0
        self.udp_first_publish_monotonic = None
        self.udp_last_publish_monotonic = None

    def _deliver_udp_pending(self):
        run_id = self.udp_pending_run_id
        if run_id is None:
            return
        now = time.monotonic()
        if (self.udp_pending_received_monotonic is not None and
                now - self.udp_pending_received_monotonic >
                self.subscriber_wait_timeout_s and
                self.mission_start_publisher.get_subscription_count() == 0):
            self.get_logger().warning(
                'pending UDP START still has no mission subscriber')
            self.udp_pending_received_monotonic = now
        if self.mission_start_publisher.get_subscription_count() == 0:
            return
        if (self.udp_last_publish_monotonic is not None and
                now - self.udp_last_publish_monotonic <
                self.trigger_repeat_period_s):
            return
        message = Bool(data=True)
        self.mission_start_publisher.publish(message)
        self.udp_publish_count += 1
        if self.udp_first_publish_monotonic is None:
            self.udp_first_publish_monotonic = now
        self.udp_last_publish_monotonic = now
        if self.udp_publish_count >= self.trigger_repeat_count:
            if self.udp_journal.commit(run_id):
                self.udp_pending_run_id = None
                self.udp_pending_source = None
                self.udp_pending_received_monotonic = None

    def _publish_link(self, alive):
        alive = bool(alive)
        if alive == self.last_link_state:
            return
        self.last_link_state = alive
        self.link_publisher.publish(Bool(data=alive))

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
        now_monotonic = time.monotonic()
        if (self.udp_socket is not None and self.udp_peer_host and
                1 <= self.udp_peer_port <= 65535 and
                now_monotonic - self.last_ping_monotonic >= 1.0):
            try:
                self.udp_socket.sendto(
                    b'PING', (self.udp_peer_host, self.udp_peer_port))
                self.last_ping_monotonic = now_monotonic
            except OSError as error:
                self.get_logger().warning('UDP PING failed: %s' % error)
        if (self.last_valid_udp_monotonic is not None and
                now_monotonic - self.last_valid_udp_monotonic >
                self.heartbeat_timeout_s):
            self._publish_link(False)
        connected = (heartbeat_is_current(
            self.last_heartbeat_ns, now_ns, self.heartbeat_timeout_s) or
                     (self.last_valid_udp_monotonic is not None and
                      now_monotonic - self.last_valid_udp_monotonic <=
                      self.heartbeat_timeout_s))
        output = String()
        output.data = json.dumps({
            'transport': self.transport.kind,
            'connected': connected,
            'stamp_ns': now_ns,
            'last_frame_source': self.last_frame_source,
            'last_frame_sequence': self.last_frame_sequence,
            'last_frame_stamp_ns': self.last_frame_stamp_ns,
            'pending_run_id': self.udp_pending_run_id,
            'pending_source': self.udp_pending_source,
            'pending_received_monotonic':
                self.udp_journal.pending_received_monotonic,
            'publish_count': self.udp_publish_count,
            'first_publish_monotonic': self.udp_first_publish_monotonic,
            'last_publish_monotonic': self.udp_last_publish_monotonic,
            'first_publish_delay_s': (
                None if self.udp_first_publish_monotonic is None or
                self.udp_request_received_monotonic is None else
                self.udp_first_publish_monotonic -
                self.udp_request_received_monotonic),
        }, separators=(',', ':'))
        self.transport_status_publisher.publish(output)


def main(args=None):
    """Run the car start gateway."""
    rclpy.init(args=args)
    node = CarStartGateway()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
