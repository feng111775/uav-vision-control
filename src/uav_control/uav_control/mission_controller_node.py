"""ROS/PX4 v1.16 adapter for the unified D-task mission logic."""

import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleAttitude, VehicleCommand, VehicleCommandAck,
                          VehicleLocalPosition, VehicleStatus)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8
from uav_vision import d_task_schema as vision_schema

from .mission_logic import MissionLogic
from .mission_schema import PX4_LOCAL_POSITION_TOPIC
from .mission_schema import state_allows_flight_setpoint
from .mission_schema import STATE_ID, TELEMETRY, TELEMETRY_LENGTH
from .px4_command_tracker import CommandTracker
from .touchdown_detector import TouchdownDetector
from .visual_guidance import VisualGuidance


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')
        defaults = {
            'mission_mode': 'hover_test', 'target_altitude': 1.0,
            'simulation_mode': False, 'competition_mode': False,
            'enable_control': False, 'enable_auto_arm': False,
            'enable_visual_follow': False, 'enable_payload_release': False,
            'enable_dynamic_landing': False, 'enable_second_takeoff': False,
            'prestream_seconds': 1.0, 'hover_confirm_seconds': 3.0,
            'hover_test_seconds': 10.0, 'dwell_on_car_seconds': 5.0,
            'mission_timeout_seconds': 90.0, 'b_deadline_seconds': 15.0,
            'altitude_tolerance': 0.1, 'stable_seconds': 1.0,
            'status_timeout': 1.0, 'position_timeout': 0.5,
            'attitude_timeout': 0.5, 'safety_timeout': 1.0,
            'car_timeout': 2.0, 'max_tilt_rad': 0.45,
            'control_rate_hz': 20.0, 'align_error': 0.10,
            'descent_speed': 0.25, 'near_descent_speed': 0.10,
            'command_timeout': 1.0, 'command_max_attempts': 3}
        defaults['payload_ack_timeout'] = 3.0
        defaults['touchdown_verify_seconds'] = 1.0
        defaults['target_loss_abort_seconds'] = 1.0
        defaults['kinematic_touchdown_height_tolerance'] = 0.0
        defaults['sitl_nav_land_after_touchdown'] = False
        defaults['guidance_kp_forward'] = 0.3
        defaults['guidance_kp_left'] = 0.3
        defaults['guidance_max_speed'] = 0.5
        defaults['camera_x_sign'] = -1.0
        defaults['camera_y_sign'] = -1.0
        defaults['dry_run_px4_commands'] = False
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        logic_names = ('mission_mode', 'target_altitude', 'simulation_mode',
                       'competition_mode', 'enable_control', 'enable_auto_arm',
                       'enable_visual_follow', 'enable_payload_release',
                       'enable_dynamic_landing', 'enable_second_takeoff',
                       'prestream_seconds', 'hover_confirm_seconds',
                       'hover_test_seconds', 'dwell_on_car_seconds',
                       'mission_timeout_seconds', 'b_deadline_seconds',
                       'altitude_tolerance', 'stable_seconds')
        logic_names = logic_names + (
            'payload_ack_timeout', 'touchdown_verify_seconds')
        self.logic = MissionLogic(**{n: self.get_parameter(n).value for n in logic_names})
        self.guidance = VisualGuidance(
            kp_forward=self.get_parameter('guidance_kp_forward').value,
            kp_left=self.get_parameter('guidance_kp_left').value,
            max_speed=self.get_parameter('guidance_max_speed').value,
            camera_x_sign=self.get_parameter('camera_x_sign').value,
            camera_y_sign=self.get_parameter('camera_y_sign').value)
        self.touchdown = TouchdownDetector()
        timeout = self.get_parameter('command_timeout').value
        attempts = self.get_parameter('command_max_attempts').value
        self.trackers = {name: CommandTracker(timeout, attempts) for name in
                         ('mode', 'arm', 'disarm', 'land')}
        self.last_status = self.last_position = self.last_attitude = None
        self.last_safety = self.last_car = None
        self.roll = self.pitch = self.yaw = 0.0
        self.error = [0.0] * vision_schema.LANDING_ERROR_LENGTH
        self.tracked = vision_schema.invalid_tracked()
        self.touchdown_sensor = False
        self.path = Path()
        self.path.header.frame_id = 'map_ned'
        self.last_state = None
        self.last_event = None
        self.payload_pulse_sent = False
        self.target_loss_since = None
        self.last_setpoint_mode = 'position'
        self.dry_run_px4_commands = bool(
            self.get_parameter('dry_run_px4_commands').value)
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.state_pub = self.create_publisher(String, '/uav/mission/state', 10)
        self.event_pub = self.create_publisher(String, '/uav/mission/event', 10)
        self.telemetry_pub = self.create_publisher(Float32MultiArray, '/uav/mission/telemetry', 10)
        self.path_pub = self.create_publisher(Path, '/uav/mission/path', 10)
        self.payload_pub = self.create_publisher(Bool, '/uav/payload/release', 10)
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._status,
            px4_qos)
        self.create_subscription(
            VehicleLocalPosition,
            PX4_LOCAL_POSITION_TOPIC,
            self._position,
            px4_qos)
        self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude,
            px4_qos)
        self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self._ack,
            px4_qos)
        self.create_subscription(Bool, '/car/mission_start', self._start, 10)
        self.create_subscription(UInt8, '/car/progress', self._car, 10)
        self.create_subscription(Bool, '/uav/safety/ready', self._safety, 10)
        self.create_subscription(Bool, '/uav/mission/reset', self._reset, 10)
        self.create_subscription(Bool, '/uav/touchdown_sensor', self._touchdown_sensor, 10)
        self.create_subscription(Bool, '/uav/payload/release_ack', self._payload_ack, 10)
        self.create_subscription(Float32MultiArray, '/vision/landing_error', self._error, 10)
        self.create_subscription(Float32MultiArray, '/vision/target/tracked', self._tracked, 10)
        self.create_timer(1.0 / self.get_parameter('control_rate_hz').value, self._timer)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def _status(self, msg):
        self.last_status = self.now()
        self.logic.update_status(msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                                 msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                                 msg.failsafe)

    def _position(self, msg):
        self.last_position = self.now()
        valid = msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid and (
            msg.heading_good_for_control or self.logic.simulation_mode)
        self.logic.update_position(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading, valid)
        if valid:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map_ned'
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = msg.x, msg.y, msg.z
            pose.pose.orientation.w = 1.0
            self.path.poses.append(pose)
            self.path.poses = self.path.poses[-2000:]

    def _attitude(self, msg):
        self.last_attitude = self.now()
        w, x, y, z = [float(v) for v in msg.q]
        self.roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        self.pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        self.logic.attitude_valid = all(math.isfinite(v)
                                        for v in (self.roll, self.pitch, self.yaw))
        self.logic.abnormal_tilt = max(abs(self.roll), abs(
            self.pitch)) > self.get_parameter('max_tilt_rad').value

    def _ack(self, msg):
        for tracker in self.trackers.values():
            if tracker.acknowledge(msg.command, msg.result):
                self.logic.event = 'COMMAND_ACK_%s_%s' % (msg.command, tracker.status)
                if tracker.failed:
                    self.logic.transition('FAILSAFE_LAND', self.now(), self.logic.event)
                break

    def _start(self, msg):
        self.logic.start_signal = bool(msg.data)

    def _car(self, msg):
        self.last_car = self.now()
        self.logic.update_car_progress(msg.data)

    def _safety(self, msg):
        self.last_safety = self.now()
        self.logic.safety_ready = bool(msg.data)

    def _reset(self, msg):
        if msg.data:
            self.logic.reset_if_safe()

    def _touchdown_sensor(self, msg): self.touchdown_sensor = bool(msg.data)
    def _payload_ack(self, msg): self.logic.payload_ack = bool(msg.data)

    def _error(self, msg):
        try:
            self.error = vision_schema.validate_landing_error(msg.data)
        except ValueError:
            self.error = vision_schema.invalid_landing_error()

    def _tracked(self, msg):
        try:
            self.tracked = vision_schema.validate_tracked(msg.data)
        except ValueError:
            self.tracked = vision_schema.invalid_tracked()

    def _publish_command(self, name, command, now, param1=0.0, param2=0.0):
        tracker = self.trackers[name]
        if not tracker.request(command, now):
            if tracker.failed:
                self.logic.transition(
                    'FAILSAFE_LAND', now, 'COMMAND_%s_%s' %
                    (name, tracker.status))
            return
        msg = VehicleCommand()
        msg.timestamp = self.timestamp()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        if self.dry_run_px4_commands:
            self.logic.event = 'DRY_RUN_COMMAND_%s' % name.upper()
            return
        self.command_pub.publish(msg)

    def _publish_control(self, mode, position=None, velocity=None):
        heartbeat = OffboardControlMode()
        heartbeat.timestamp = self.timestamp()
        heartbeat.position = mode == 'position'
        heartbeat.velocity = mode == 'velocity'
        heartbeat.acceleration = heartbeat.attitude = heartbeat.body_rate = False
        heartbeat.thrust_and_torque = heartbeat.direct_actuator = False
        self.offboard_pub.publish(heartbeat)
        nan = float('nan')
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = self.timestamp()
        setpoint.position = list(position) if position is not None else [nan] * 3
        setpoint.velocity = list(velocity) if velocity is not None else [nan] * 3
        setpoint.acceleration = [nan] * 3
        setpoint.jerk = [nan] * 3
        setpoint.yaw = self.logic.h[3] if self.logic.h else nan
        setpoint.yawspeed = nan
        self.setpoint_pub.publish(setpoint)
        self.last_setpoint_mode = mode

    def _timer(self):
        now = self.now()
        status_timeout = self.get_parameter('status_timeout').value
        position_timeout = self.get_parameter('position_timeout').value
        attitude_timeout = self.get_parameter('attitude_timeout').value
        status_stale = (self.last_status is None or
                        now - self.last_status > status_timeout)
        position_stale = (self.last_position is None or
                          now - self.last_position > position_timeout)
        if status_stale or position_stale:
            self.logic.px4_fresh = False
        if self.last_attitude is None or now - self.last_attitude > attitude_timeout:
            self.logic.attitude_valid = False
        if not self.logic.simulation_mode and (
                self.last_safety is None or now -
                self.last_safety > self.get_parameter('safety_timeout').value):
            self.logic.safety_ready = False
        valid = self.error[vision_schema.VALID] == 1.0
        confidence = self.error[vision_schema.ERROR_CONFIDENCE]
        age = self.error[vision_schema.ERROR_TARGET_AGE_MS]
        ex = self.error[vision_schema.ERROR_X_NORMALIZED]
        ey = self.error[vision_schema.ERROR_Y_NORMALIZED]
        self.logic.target_ok = valid and confidence >= 60.0 and age <= 250.0
        self.logic.aligned = self.logic.target_ok and abs(ex) <= self.get_parameter(
            'align_error').value and abs(ey) <= self.get_parameter('align_error').value
        if self.logic.state == 'DESCEND_ON_CAR' and not self.logic.target_ok:
            if self.target_loss_since is None:
                self.target_loss_since = now
            elif now - self.target_loss_since >= self.get_parameter(
                    'target_loss_abort_seconds').value:
                self.logic.transition('ABORT_RETURN_H', now,
                                      'TARGET_LOST_DURING_DESCENT')
        else:
            self.target_loss_since = None
        if (self.logic.state == 'DESCEND_ON_CAR' and
                (self.last_car is None or
                 now - self.last_car > self.get_parameter('car_timeout').value)):
            self.logic.transition('ABORT_RETURN_H', now, 'CAR_DATA_TIMEOUT')
        if self.logic.state == 'DESCEND_ON_CAR':
            kinematic = (self.logic.position is not None and
                         self.logic.position[2] >= self.logic.h[2] -
                         self.get_parameter(
                             'kinematic_touchdown_height_tolerance').value)
            self.logic.touchdown = self.touchdown.update(
                now,
                self.touchdown_sensor,
                kinematic,
                self.logic.velocity[2],
                self.roll,
                self.pitch)
        elif self.logic.state not in (
                'TOUCHDOWN_VERIFY', 'DISARM_ON_CAR', 'DWELL_ON_CAR'):
            self.logic.touchdown = False
        self.logic.step(now)
        if self.logic.state == 'SECOND_PRESTREAM' and self.last_state != self.logic.state:
            for name in ('mode', 'arm', 'land', 'disarm'):
                self.trackers[name].reset()
            self.payload_pulse_sent = False
        if self.logic.enable_control and self.logic.state not in (
            'WAIT_PX4',
            'WAIT_SAFETY',
            'WAIT_START',
            'COMPLETE',
            'EXTERNAL_CONTROL',
                'FAILSAFE_LAND'):
            if self.logic.state == 'REQUEST_OFFBOARD':
                self._publish_command(
                    'mode', VehicleCommand.VEHICLE_CMD_DO_SET_MODE, now, 1.0, 6.0)
            if self.logic.state in ('ARMING', 'SECOND_ARM') and self.logic.auto_arm_allowed():
                self._publish_command(
                    'arm', VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, now, 1.0)
            if self.logic.state == 'DISARM_ON_CAR':
                if self.get_parameter(
                        'sitl_nav_land_after_touchdown').value:
                    self._publish_command(
                        'land', VehicleCommand.VEHICLE_CMD_NAV_LAND, now)
                else:
                    self._publish_command(
                        'disarm',
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        now, 0.0)
            if self.logic.state == 'LAND_H':
                self._publish_command('land', VehicleCommand.VEHICLE_CMD_NAV_LAND, now)
            if self.logic.state in (
                'FOLLOW_TARGET',
                'DROP_ALIGN',
                'LANDING_ALIGN',
                    'DESCEND_ON_CAR', 'TOUCHDOWN_VERIFY', 'DISARM_ON_CAR'):
                n, e = self.guidance.velocity(
                    valid, ex, ey, confidence, age, self.logic.heading)
                down = 0.0
                if self.logic.state == 'DESCEND_ON_CAR' and self.logic.aligned:
                    down = self.get_parameter('descent_speed').value
                elif self.logic.state in (
                        'TOUCHDOWN_VERIFY', 'DISARM_ON_CAR'):
                    n = e = 0.0
                    down = self.get_parameter('near_descent_speed').value
                if not (self.logic.state == 'DISARM_ON_CAR' and
                        self.get_parameter(
                            'sitl_nav_land_after_touchdown').value):
                    self._publish_control('velocity', velocity=(n, e, down))
            elif (self.logic.h is not None and
                  state_allows_flight_setpoint(self.logic.state)):
                x, y = self.logic.h[0], self.logic.h[1]
                if (self.logic.second_cycle and
                        self.logic.state in (
                            'SECOND_PRESTREAM', 'REQUEST_OFFBOARD',
                            'SECOND_ARM', 'SECOND_TAKEOFF') and
                        self.logic.second_takeoff_origin is not None):
                    x, y = self.logic.second_takeoff_origin[:2]
                self._publish_control(
                    'position',
                    position=(
                        x,
                        y,
                        self.logic.cruise_z))
        elif (self.logic.enable_control and
              self.logic.state == 'FAILSAFE_LAND' and
              self.last_status is not None):
            self._publish_command('land', VehicleCommand.VEHICLE_CMD_NAV_LAND, now)
        if self.logic.payload_sent and not self.payload_pulse_sent:
            pulse = Bool()
            pulse.data = True
            self.payload_pub.publish(pulse)
            self.payload_pulse_sent = True
        self._publish_observability(now)
        self.last_state = self.logic.state

    def _publish_observability(self, now):
        state = String()
        state.data = self.logic.state
        self.state_pub.publish(state)
        event = String()
        event.data = self.logic.event
        self.event_pub.publish(event)
        data = [0.0] * TELEMETRY_LENGTH
        elapsed = 0.0 if self.logic.started_at is None else now - self.logic.started_at
        values = {'mission_active': self.logic.state not in ('WAIT_PX4',
                                                             'WAIT_SAFETY',
                                                             'WAIT_START',
                                                             'COMPLETE'),
                  'mission_mode': int(self.logic.mode),
                  'state_id': STATE_ID[self.logic.state],
                  'elapsed_seconds': elapsed,
                  'heading': self.logic.heading,
                  'target_valid': self.logic.target_ok,
                  'target_error_x': self.error[1],
                  'target_error_y': self.error[2],
                  'target_confidence': self.error[6],
                  'car_progress': int(self.logic.car_progress),
                  'armed': self.logic.armed,
                  'offboard_active': self.logic.offboard,
                  'failsafe': self.logic.failsafe,
                  'command_ack_status': max(t.attempts for t in self.trackers.values()),
                  'touchdown_confirmed': self.logic.touchdown}
        if self.logic.position:
            for key, value in zip(('x', 'y', 'z', 'vx', 'vy', 'vz'),
                                  self.logic.position + self.logic.velocity):
                values[key] = value
            if self.logic.h:
                values['h_distance'] = math.hypot(
                    self.logic.position[0] - self.logic.h[0],
                    self.logic.position[1] - self.logic.h[1])
        for key, value in values.items():
            data[TELEMETRY[key]] = float(value)
        telemetry = Float32MultiArray()
        telemetry.data = data
        self.telemetry_pub.publish(telemetry)
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
