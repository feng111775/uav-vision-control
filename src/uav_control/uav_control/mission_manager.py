"""ROS adapter for the pure stage 4C competition mission flow."""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .stage4c_core import (MarkerObservation, MissionConfig, MissionFlow,
                           MissionState)


class MissionManager(Node):
    """Publish high-level commands; never publish a PX4 message."""

    def __init__(self):
        super().__init__('mission_manager')
        defaults = MissionConfig()
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        self.declare_parameter('simulation_mode', True)
        config = MissionConfig(**{
            name: self.get_parameter(name).value for name in vars(defaults)})
        self.flow = MissionFlow(config)
        self.position = (0.0, 0.0, 0.0)
        self.px4_ok = True
        self.failsafe = False
        self.landed = False
        self.intercept_reached = False
        self.observation = None
        self.payload_result = None
        self.last_state = self.flow.state
        self.command_pub = self.create_publisher(
            String, '/uav_mission/control/command', 10)
        self.release_pub = self.create_publisher(
            String, '/uav_mission/payload/request', 10)
        self.state_pub = self.create_publisher(
            String, '/uav_mission/state', 10)
        self.create_subscription(
            String, '/uav_mission/events/start', self._start, 10)
        self.create_subscription(
            String, '/uav_mission/vision/marker', self._marker, 10)
        self.create_subscription(
            String, '/uav_mission/payload/result', self._payload, 10)
        self.create_subscription(
            Float32MultiArray, '/uav_mission/sim/position', self._position, 10)
        self.create_subscription(
            String, '/uav_mission/sim/px4_status', self._px4, 10)
        now = self.now()
        self.flow.initialize(now)
        self._log_transition(MissionState.INITIALIZING, self.flow.state)
        self.create_timer(0.05, self._tick)

    def now(self):
        """Return ROS time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _start(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if event.get('valid') and self.flow.start(
                str(event.get('task_id', '')), self.now(), self.position):
            self._log_transition(MissionState.WAIT_FOR_START, self.flow.state)

    def _marker(self, msg):
        try:
            data = json.loads(msg.data)
            self.observation = MarkerObservation(
                stamp=float(data['stamp_ns']) * 1e-9,
                detected=bool(data['detected']),
                confidence=float(data['confidence']),
                error_x=float(data['error_x']),
                error_y=float(data['error_y']),
                relative_x=data.get('relative_x'),
                relative_y=data.get('relative_y'),
                velocity_x=data.get('velocity_x'),
                velocity_y=data.get('velocity_y'))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.observation = None

    def _payload(self, msg):
        try:
            result = json.loads(msg.data)
            if result.get('task_id') == self.flow.task_id:
                self.payload_result = result.get('status')
        except json.JSONDecodeError:
            pass

    def _position(self, msg):
        if len(msg.data) >= 3:
            self.position = tuple(float(v) for v in msg.data[:3])
        if len(msg.data) >= 4:
            self.intercept_reached = bool(msg.data[3])

    def _px4(self, msg):
        try:
            status = json.loads(msg.data)
            self.px4_ok = bool(status.get('px4_ok', True))
            self.failsafe = bool(status.get('failsafe', False))
            self.landed = bool(status.get('landed', False))
        except json.JSONDecodeError:
            self.px4_ok = False

    def _log_transition(self, old, new):
        elapsed = (0.0 if self.flow.task_started is None else
                   self.now() - self.flow.task_started)
        self.get_logger().info(
            '%s -> %s reason=%s elapsed=%.2fs' %
            (old.value, new.value, self.flow.last_reason, elapsed))

    def _tick(self):
        now = self.now()
        old = self.flow.state
        self.flow.update(
            now, self.position, self.px4_ok, self.failsafe,
            self.observation, self.intercept_reached,
            self.payload_result, self.landed)
        if self.flow.state != old:
            self._log_transition(old, self.flow.state)
        command = String()
        command_data = self.flow.command(now)
        command_data['phase'] = self.flow.state.value
        command.data = json.dumps(command_data, separators=(',', ':'))
        self.command_pub.publish(command)
        if (self.flow.state == MissionState.RELEASE_PAYLOAD and
                self.payload_result is None):
            request = String()
            request.data = json.dumps({
                'task_id': self.flow.task_id,
                'release_id': self.flow.task_id + '-release-1',
                'stamp_ns': self.get_clock().now().nanoseconds,
                'allowed': (not self.failsafe and
                            self.flow.payload_requested),
            }, separators=(',', ':'))
            self.release_pub.publish(request)
        state = String()
        state.data = json.dumps({
            'task_id': self.flow.task_id,
            'stamp_ns': self.get_clock().now().nanoseconds,
            'state': self.flow.state.value,
            'reason': self.flow.last_reason,
            'elapsed_s': (0.0 if self.flow.task_started is None else
                          now - self.flow.task_started),
            'predicted_car_distance_m': self.flow.predicted_distance(now),
        }, separators=(',', ':'))
        self.state_pub.publish(state)


def main(args=None):
    """Run the stage 4C manager."""
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
