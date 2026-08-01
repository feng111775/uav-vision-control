"""Inject only car/vision inputs and record real PX4 SITL evidence."""

import json
import math
from pathlib import Path

from px4_msgs.msg import (VehicleCommandAck, VehicleLandDetected,
                          VehicleLocalPosition, VehicleStatus)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .px4_qos import px4_output_qos
from .stage4c_core import StartProtocol


class SitlAcceptanceDriver(Node):
    """Drive one task without publishing simulated PX4 state."""

    def __init__(self):
        super().__init__('sitl_acceptance_driver')
        self.declare_parameter('task_id', 'stage4c-sitl')
        self.declare_parameter('result_path', '/tmp/stage4c_sitl_result.json')
        self.declare_parameter('exercise_vision_loss', False)
        self.declare_parameter('marker_delay_s', 3.0)
        self.declare_parameter('suppress_marker', False)
        self.declare_parameter('vision_loss_mode', 'none')
        self.declare_parameter('reacquire_after_long_loss', False)
        self.declare_parameter('never_align', False)
        self.declare_parameter('duplicate_start_count', 1)
        self.declare_parameter('duplicate_release_count', 0)
        self.task_id = str(self.get_parameter('task_id').value)
        self.start_id = 'start-1'
        self.protocol_task_id = self.task_id + ':' + self.start_id
        self.result_path = Path(self.get_parameter('result_path').value)
        self.exercise_vision_loss = bool(
            self.get_parameter('exercise_vision_loss').value)
        self.marker_delay_s = float(
            self.get_parameter('marker_delay_s').value)
        self.suppress_marker = bool(
            self.get_parameter('suppress_marker').value)
        self.vision_loss_mode = str(
            self.get_parameter('vision_loss_mode').value)
        self.reacquire_after_long_loss = bool(
            self.get_parameter('reacquire_after_long_loss').value)
        self.never_align = bool(self.get_parameter('never_align').value)
        self.duplicate_start_count = int(
            self.get_parameter('duplicate_start_count').value)
        self.duplicate_release_count = int(
            self.get_parameter('duplicate_release_count').value)
        self.state = ''
        self.start_time = None
        self.state_entered = None
        self.sent_start = False
        self.last_start_send = None
        self.last_heartbeat_send = None
        self.heartbeat_seen = False
        self.position = None
        self.home = None
        self.positions = []
        self.timeline = []
        self.acks = []
        self.failsafe_seen = False
        self.offboard_seen = False
        self.landed = True
        self.release_results = []
        self.first_follow_time = None
        self.search_start_time = None
        self.vision_events = []
        self.vision_event_names = set()
        self.release_request = None
        self.release_duplicated = False
        self.start_pub = self.create_publisher(
            String, '/uav_mission/sim/car_start', 10)
        self.marker_pub = self.create_publisher(
            Float32MultiArray, '/uav_mission/sim/marker', 10)
        self.release_duplicate_pub = self.create_publisher(
            String, '/uav_mission/payload/request', 10)
        self.create_subscription(
            String, '/uav_mission/state', self._state, 10)
        self.create_subscription(
            String, '/uav_mission/events/start_result',
            self._start_result, 10)
        self.create_subscription(
            String, '/uav_mission/payload/result', self._release, 10)
        self.create_subscription(
            String, '/uav_mission/payload/request',
            self._release_request, 10)
        qos = px4_output_qos()
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._position, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self._status, qos)
        self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self._ack, qos)
        self.create_subscription(
            VehicleLandDetected, '/fmu/out/vehicle_land_detected',
            self._land, qos)
        self.create_timer(0.1, self._tick)

    def now(self):
        """Return ROS wall time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def _state(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        new_state = str(data.get('state', ''))
        if new_state and new_state != self.state:
            now = self.now()
            self.state = new_state
            self.state_entered = now
            self.timeline.append({
                'state': new_state,
                'time_s': now,
                'elapsed_s': data.get('elapsed_s', 0.0),
                'reason': data.get('reason', ''),
                'position': self.position,
            })
            if new_state == 'TAKEOFF' and self.home is None:
                self.home = self.position
                self.start_time = now
            if new_state == 'FOLLOW_CAR' and self.first_follow_time is None:
                self.first_follow_time = now
            if new_state == 'SEARCH_CAR' and self.search_start_time is None:
                self.search_start_time = now
            if new_state == 'COMPLETE':
                self._finish()

    def _position(self, msg):
        values = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
        if (msg.xy_valid and msg.z_valid and
                all(math.isfinite(value) for value in values)):
            self.position = [float(msg.x), float(msg.y), float(msg.z)]
            if self.start_time is not None:
                self.positions.append({
                    'elapsed_s': self.now() - self.start_time,
                    'x': float(msg.x), 'y': float(msg.y), 'z': float(msg.z),
                    'vx': float(msg.vx), 'vy': float(msg.vy),
                    'vz': float(msg.vz),
                })

    def _status(self, msg):
        self.failsafe_seen = self.failsafe_seen or bool(msg.failsafe)
        self.offboard_seen = self.offboard_seen or (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def _ack(self, msg):
        self.acks.append({
            'time_s': self.now(),
            'command': int(msg.command),
            'result': int(msg.result),
        })

    def _land(self, msg):
        self.landed = bool(msg.landed)

    def _release(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get('task_id') == self.protocol_task_id:
                self.release_results.append(data)
        except json.JSONDecodeError:
            pass

    def _release_request(self, msg):
        if not self.release_duplicated:
            self.release_request = msg.data

    def _start_result(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if (result.get('session_id') != self.task_id or
                result.get('start_id') != self.start_id):
            return
        if result.get('type') == 'HEARTBEAT':
            self.heartbeat_seen = True
            return
        if result.get('type') != 'START_ACCEPTED' or self.sent_start:
            return
        self.sent_start = True
        if self.duplicate_start_count > 1:
            duplicate = String()
            duplicate.data = json.dumps(StartProtocol.build(
                self.task_id, self.start_id, 1,
                self.get_clock().now().nanoseconds))
            for _ in range(self.duplicate_start_count - 1):
                self.start_pub.publish(duplicate)

    def _tick(self):
        if (self.state not in ('COMPLETE', '') and self.position is not None and
                (self.last_heartbeat_send is None or
                 self.now() - self.last_heartbeat_send >= 0.5)):
            heartbeat = String()
            heartbeat.data = json.dumps(StartProtocol.build(
                self.task_id, self.start_id, 0,
                self.get_clock().now().nanoseconds, command='HEARTBEAT'))
            self.start_pub.publish(heartbeat)
            self.last_heartbeat_send = self.now()
        if (self.state == 'WAIT_FOR_START' and self.position is not None and
                self.heartbeat_seen and not self.sent_start and
                (self.last_start_send is None or
                 self.now() - self.last_start_send >= 0.5)):
            output = String()
            output.data = json.dumps(StartProtocol.build(
                self.task_id, self.start_id, 1,
                self.get_clock().now().nanoseconds))
            self.start_pub.publish(output)
            self.last_start_send = self.now()
        if (self.release_request is not None and
                not self.release_duplicated and
                self.duplicate_release_count > 0):
            self.release_duplicated = True
            duplicate = String()
            duplicate.data = self.release_request
            for _ in range(self.duplicate_release_count):
                # Publish the exact same release_id through the formal topic.
                self.release_duplicate_pub.publish(duplicate)
        publish_marker = self.state in (
            'SEARCH_CAR', 'ACQUIRE_CAR', 'FOLLOW_CAR', 'DROP_ALIGN')
        if self.state in ('SEARCH_CAR', 'ACQUIRE_CAR'):
            publish_marker = (
                not self.suppress_marker and
                self.search_start_time is not None and
                self.now() - self.search_start_time >= self.marker_delay_s)
        error = 0.0
        loss_mode = (
            'long' if self.exercise_vision_loss else self.vision_loss_mode)
        if loss_mode in ('short', 'long') and \
                self.first_follow_time is not None:
            loss_elapsed = self.now() - self.first_follow_time
            if loss_elapsed < 0.5:
                error = 0.15
            elif loss_elapsed < 1.2:
                publish_marker = False
            elif loss_mode == 'short':
                error = 0.15 if loss_elapsed < 2.0 else 0.0
            elif loss_elapsed < 1.5:
                error = 0.15
            elif loss_elapsed < 4.5:
                publish_marker = False
            elif (loss_mode == 'long' and
                  not self.reacquire_after_long_loss):
                publish_marker = False
            thresholds = [
                (0.5, 'short_loss_start'),
                (1.2, 'short_loss_end'),
            ]
            if loss_mode == 'long':
                thresholds.extend([
                    (1.5, 'long_loss_start'),
                    (4.5, 'long_loss_end'),
                ])
            for threshold, name in thresholds:
                if loss_elapsed >= threshold and name not in \
                        self.vision_event_names:
                    self.vision_event_names.add(name)
                    self.vision_events.append({
                        'event': name,
                        'elapsed_since_follow_s': loss_elapsed,
                        'mission_state': self.state,
                        'position': self.position,
                    })
        elif self.state == 'FOLLOW_CAR':
            elapsed = self.now() - (self.state_entered or self.now())
            error = 0.15 if self.never_align or elapsed < 1.0 else 0.0
        if publish_marker:
            marker = Float32MultiArray()
            marker.data = [1.0, 0.95, error, 0.0]
            self.marker_pub.publish(marker)

    def _finish(self):
        total = 0.0 if self.start_time is None else self.now() - self.start_time
        home_error = None
        if self.home is not None and self.position is not None:
            home_error = math.dist(self.home[:2], self.position[:2])
        result = {
            'task_id': self.task_id,
            'timeline': self.timeline,
            'home': self.home,
            'final_position': self.position,
            'home_horizontal_error_m': home_error,
            'total_time_s': total,
            'failsafe_seen': self.failsafe_seen,
            'offboard_seen': self.offboard_seen,
            'landed': self.landed,
            'release_results': self.release_results,
            'acks': self.acks,
            'vision_events': self.vision_events,
            'position_samples': self.positions,
        }
        self.result_path.parent.mkdir(parents=True, exist_ok=True)
        self.result_path.write_text(
            json.dumps(result, indent=2, sort_keys=True) + '\n')
        self.get_logger().info('acceptance result written: %s' %
                               self.result_path)
        rclpy.shutdown()


def main(args=None):
    """Run one acceptance scenario."""
    rclpy.init(args=args)
    node = SitlAcceptanceDriver()
    rclpy.spin(node)
    node.destroy_node()
