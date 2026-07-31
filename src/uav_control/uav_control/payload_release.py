"""Dry-run one-shot payload release node."""

import json
import os
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .payload_backend import (DryRunPayloadBackend, GpioPayloadBackend,
                              ReleaseConfig, validate_release_config)
from .stage4c_core import PersistentPayloadGate


class DryRunReleaseActuator:
    """Software-only actuator that explicitly performs no physical action."""

    actuator_type = 'DRY_RUN'

    def execute(self, requested_result):
        """Return a software result with an explicit physical-action flag."""
        result = str(requested_result).upper()
        if result == 'SUCCESS':
            result = 'DRY_RUN_CONFIRMED'
        if result not in ('DRY_RUN_CONFIRMED', 'FAILED', 'TIMEOUT'):
            result = 'FAILED'
        return result, False


class PhysicalReleaseActuator:
    """Non-constructible placeholder until hardware is specified and reviewed."""

    def __init__(self):
        raise RuntimeError('physical release actuator is not implemented')


class PayloadRelease(Node):
    """Reject unsafe/repeated requests and never access GPIO in dry-run."""

    def __init__(self):
        super().__init__('payload_release')
        self.declare_parameter('dry_run', True)
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('enable_payload_release', False)
        self.declare_parameter('simulated_result', 'SUCCESS')
        self.declare_parameter('physical_release_enabled', False)
        self.declare_parameter('payload_backend', 'dry_run')
        self.declare_parameter('payload_gpio', 18)
        self.declare_parameter('payload_pwm_frequency_hz', 50)
        self.declare_parameter('payload_neutral_pulse_us', 1500)
        self.declare_parameter('payload_release_pulse_us', 1900)
        self.declare_parameter('payload_hold_seconds', 0.8)
        self.declare_parameter('payload_reset_seconds', 0.8)
        self.release_config = ReleaseConfig(
            gpio=int(self.get_parameter('payload_gpio').value),
            pwm_frequency_hz=int(self.get_parameter('payload_pwm_frequency_hz').value),
            neutral_pulse_us=int(self.get_parameter('payload_neutral_pulse_us').value),
            release_pulse_us=int(self.get_parameter('payload_release_pulse_us').value),
            hold_seconds=float(self.get_parameter('payload_hold_seconds').value),
            reset_seconds=float(self.get_parameter('payload_reset_seconds').value))
        validate_release_config(self.release_config)
        self.declare_parameter(
            'payload_state_path', os.path.join(
                tempfile.gettempdir(), 'uav_mission_payload_state.json'))
        if not self.get_parameter('dry_run').value:
            raise ValueError('payload_release supports dry_run only')
        if self.get_parameter('physical_release_enabled').value:
            raise ValueError('physical payload output is not implemented')
        backend_name = str(self.get_parameter('payload_backend').value)
        if backend_name == 'dry_run':
            self.backend = DryRunPayloadBackend()
        elif backend_name == 'gpio':
            self.backend = GpioPayloadBackend(
                bool(self.get_parameter('simulation_mode').value),
                bool(self.get_parameter('enable_payload_release').value)
                if self.has_parameter('enable_payload_release') else False)
        else:
            raise ValueError('payload_backend must be dry_run or gpio')
        self.gate = PersistentPayloadGate(
            str(self.get_parameter('payload_state_path').value))
        self.actuator = DryRunReleaseActuator()
        self.request_count = 0
        self.execution_count = 0
        self.publisher = self.create_publisher(
            String, '/uav_mission/payload/result', 10)
        self.create_subscription(
            String, '/uav_mission/payload/request', self._request, 10)
        self.create_subscription(
            String, '/uav_mission/payload/reset', self._reset, 10)

    def _request(self, msg):
        self.request_count += 1
        try:
            request = json.loads(msg.data)
            task_id = str(request['task_id'])
            session_id = str(request.get('session_id', task_id))
            release_id = str(request['release_id'])
            allowed = bool(request.get('allowed', False))
            phase = str(request.get('phase', ''))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.get_logger().warning('malformed release request rejected')
            return
        allowed = allowed and phase == 'RELEASE_PAYLOAD'
        previous = self.gate.previous_result(session_id, release_id)
        if previous is not None:
            status = previous
        else:
            try:
                status = self.gate.begin(session_id, release_id, allowed)
            except OSError:
                status = 'LOCKED'
        if status == 'EXECUTING':
            self.execution_count += 1
            requested = str(
                self.get_parameter('simulated_result').value).upper()
            requested, physical_action = self.backend.execute(
                self.release_config, requested)
            try:
                status = self.gate.finish(requested)
            except OSError:
                status = 'LOCKED'
            if status == 'DRY_RUN_CONFIRMED':
                self.get_logger().info(
                    'DRY_RUN_CONFIRMED; physical_action=false')
            else:
                self.get_logger().warning(
                    'dry-run: simulated payload result %s' % status)
        else:
            self.get_logger().warning(
                'release rejected task=%s release=%s' %
                (task_id, release_id))
        output = String()
        output.data = json.dumps({
            'task_id': task_id,
            'release_id': release_id,
            'stamp_ns': self.get_clock().now().nanoseconds,
            'status': status,
            'actuator_type': self.actuator.actuator_type,
            'physical_action': False,
            'ledger_state': self.gate.ledger_state,
            'request_count': self.request_count,
            'execution_count': self.execution_count,
            'process_execution_count': self.execution_count,
            'historical_execution_count':
                self.gate.historical_execution_count,
        }, separators=(',', ':'))
        self.publisher.publish(output)

    def _reset(self, msg):
        try:
            confirmation = str(json.loads(msg.data)['confirm'])
        except (KeyError, TypeError, json.JSONDecodeError):
            return
        if self.gate.reset_lock(confirmation):
            self.get_logger().warning('payload LOCKED state manually reset')


def main(args=None):
    """Run the dry-run payload node."""
    rclpy.init(args=args)
    node = PayloadRelease()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
