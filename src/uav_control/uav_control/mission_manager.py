"""ROS adapter for the pure stage 4C competition mission flow."""

import json
import math

from px4_msgs.msg import (VehicleLandDetected, VehicleLocalPosition,
                          VehicleStatus)
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
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
        self.declare_parameter('use_simulated_px4', True)
        config = MissionConfig(**{
            name: self.get_parameter(name).value for name in vars(defaults)})
        self.flow = MissionFlow(config)
        self.position = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.heading = None
        self.px4_ok = True
        self.failsafe = False
        self.landed = False
        self.intercept_reached = False
        self.last_position_time = None
        self.last_status_time = None
        self.armed = False
        self.offboard = False
        self.offboard_seen = False
        self.observation = None
        self.payload_result = None
        self.release_request_sent = False
        self.transport_connected = False
        self.last_transport_status_time = None
        self.last_state = self.flow.state
        self.command_pub = self.create_publisher(
            String, '/uav_mission/control/command', 10)
        self.release_pub = self.create_publisher(
            String, '/uav_mission/payload/request', 10)
        self.state_pub = self.create_publisher(
            String, '/uav_mission/state', 10)
        self.readiness_pub = self.create_publisher(
            String, '/uav_mission/readiness', 10)
        self.create_subscription(
            String, '/uav_mission/events/start', self._start, 10)
        self.create_subscription(
            String, '/uav_mission/vision/marker', self._marker, 10)
        self.create_subscription(
            String, '/uav_mission/payload/result', self._payload, 10)
        self.create_subscription(
            String, '/uav_mission/transport/status',
            self._transport_status, 10)
        self.create_subscription(
            Float32MultiArray, '/uav_mission/sim/position', self._position, 10)
        self.create_subscription(
            String, '/uav_mission/sim/px4_status', self._px4, 10)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self._vehicle_status, px4_qos)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_position, px4_qos)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._land_detected, px4_qos)
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
        healthy = self._px4_current(self.now())
        if event.get('valid') and healthy and self.flow.start(
                str(event.get('task_id', '')), self.now(), self.position):
            self._log_transition(MissionState.WAIT_FOR_START, self.flow.state)
        elif event.get('valid') and not healthy:
            self.get_logger().warning('start rejected: PX4 data not ready')

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

    def _transport_status(self, msg):
        try:
            status = json.loads(msg.data)
            self.transport_connected = bool(status['connected'])
            self.last_transport_status_time = self.now()
        except (KeyError, TypeError, json.JSONDecodeError):
            self.transport_connected = False

    def _position(self, msg):
        if not self.get_parameter('use_simulated_px4').value:
            return
        if len(msg.data) >= 3:
            self.position = tuple(float(v) for v in msg.data[:3])
        if len(msg.data) >= 4:
            self.intercept_reached = bool(msg.data[3])
        if len(msg.data) >= 5 and math.isfinite(msg.data[4]):
            self.heading = float(msg.data[4])

    def _px4(self, msg):
        if not self.get_parameter('use_simulated_px4').value:
            return
        try:
            status = json.loads(msg.data)
            self.px4_ok = bool(status.get('px4_ok', True))
            self.failsafe = bool(status.get('failsafe', False))
            self.landed = bool(status.get('landed', False))
            self.last_position_time = self.now()
            self.last_status_time = self.now()
        except json.JSONDecodeError:
            self.px4_ok = False

    def _vehicle_status(self, msg):
        if self.get_parameter('use_simulated_px4').value:
            return
        self.last_status_time = self.now()
        self.failsafe = bool(msg.failsafe)
        self.armed = (
            msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.offboard_seen = self.offboard_seen or self.offboard

    def _local_position(self, msg):
        if self.get_parameter('use_simulated_px4').value:
            return
        values = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading)
        valid = (msg.xy_valid and msg.z_valid and msg.v_xy_valid and
                 msg.v_z_valid and all(math.isfinite(v) for v in values))
        if valid:
            self.position = (float(msg.x), float(msg.y), float(msg.z))
            self.velocity = (float(msg.vx), float(msg.vy), float(msg.vz))
            self.heading = float(msg.heading)
            self.last_position_time = self.now()

    def _land_detected(self, msg):
        if not self.get_parameter('use_simulated_px4').value:
            self.landed = bool(msg.landed)

    def _px4_current(self, now):
        if self.get_parameter('use_simulated_px4').value:
            return self.px4_ok
        timeout = self.flow.config.px4_data_timeout_s
        return (
            self.last_status_time is not None and
            self.last_position_time is not None and
            now - self.last_status_time <= timeout and
            now - self.last_position_time <= timeout and
            not self.failsafe)

    def _log_transition(self, old, new):
        elapsed = (0.0 if self.flow.task_started is None else
                   self.now() - self.flow.task_started)
        self.get_logger().info(
            '%s -> %s reason=%s elapsed=%.2fs' %
            (old.value, new.value, self.flow.last_reason, elapsed))

    def _tick(self):
        now = self.now()
        self.px4_ok = self._px4_current(now)
        transport_current = (
            self.transport_connected and
            self.last_transport_status_time is not None and
            now - self.last_transport_status_time <=
            self.flow.config.px4_data_timeout_s)
        px4_ready = self.px4_ok and not self.failsafe
        readiness = String()
        readiness.data = json.dumps({
            'ready': (
                self.flow.state == MissionState.WAIT_FOR_START and
                px4_ready and transport_current and
                self.heading is not None and
                all(math.isfinite(value) for value in (
                    *self.position, self.heading))),
            'busy': self.flow.state != MissionState.WAIT_FOR_START,
            'state': self.flow.state.value,
            'stamp_ns': self.get_clock().now().nanoseconds,
        }, separators=(',', ':'))
        self.readiness_pub.publish(readiness)
        if (not self.get_parameter('use_simulated_px4').value and
                self.flow.state == MissionState.TRANSIT_TO_INTERCEPT):
            target = self.flow.command(now).get('target')
            if target:
                self.intercept_reached = (
                    math.dist(self.position, target) <=
                    self.flow.config.intercept_tolerance_m and
                    math.hypot(self.velocity[0], self.velocity[1]) <=
                    self.flow.config.search_max_speed_mps)
        offboard_lost = (
            self.offboard_seen and self.armed and not self.offboard and
            self.flow.state not in (
                MissionState.LAND, MissionState.COMPLETE,
                MissionState.EMERGENCY_LAND))
        old = self.flow.state
        self.flow.update(
            now, self.position, self.px4_ok, self.failsafe,
            self.observation, self.intercept_reached,
            self.payload_result, self.landed, offboard_lost, self.heading,
            not transport_current)
        if self.flow.state != old:
            self._log_transition(old, self.flow.state)
        command = String()
        command_data = self.flow.command(now)
        command_data['phase'] = self.flow.state.value
        command_data['task_id'] = self.flow.task_id
        command.data = json.dumps(command_data, separators=(',', ':'))
        self.command_pub.publish(command)
        if (self.flow.state == MissionState.RELEASE_PAYLOAD and
                self.payload_result is None and
                not self.release_request_sent):
            request = String()
            request.data = json.dumps({
                'task_id': self.flow.task_id,
                'session_id': self.flow.task_id.split(':', 1)[0],
                'release_id': self.flow.task_id + '-release-1',
                'stamp_ns': self.get_clock().now().nanoseconds,
                'phase': self.flow.state.value,
                'allowed': (not self.failsafe and
                            self.flow.payload_requested),
            }, separators=(',', ':'))
            self.release_pub.publish(request)
            self.release_request_sent = True
        state = String()
        state.data = json.dumps({
            'task_id': self.flow.task_id,
            'stamp_ns': self.get_clock().now().nanoseconds,
            'state': self.flow.state.value,
            'reason': self.flow.last_reason,
            'elapsed_s': (0.0 if self.flow.task_started is None else
                          now - self.flow.task_started),
            'predicted_car_distance_m': self.flow.predicted_distance(now),
            'search_heading': self.flow.search_heading,
            'search_elapsed_s': (
                0.0 if self.flow.search_started is None else
                max(0.0, now - self.flow.search_started)),
            'search_distance_m': self.flow.search_distance(self.position),
        }, separators=(',', ':'))
        self.state_pub.publish(state)


def main(args=None):
    """Run the stage 4C manager."""
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
