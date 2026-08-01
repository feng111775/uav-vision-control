"""ROS/PX4 v1.16 adapter for the unified D-task mission logic."""

import json
import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleAttitude, VehicleCommand, VehicleCommandAck,
                          VehicleLandDetected, VehicleLocalPosition,
                          VehicleStatus)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String, UInt8

from . import vision_contract as vision_schema
from .mission_logic import (control_output_allowed, data_loss_action,
                            descent_speed_for_state, MissionLogic,
                            OdomFrameGate,
                            prestream_is_safe, quaternion_is_valid,
                            vehicle_command_allowed, vision_is_fresh)
from .mission_schema import (SAFETY_BLOCK_CODES, STATE_ID, TELEMETRY,
                             TELEMETRY_LENGTH)
from .px4_command_tracker import CommandTracker
from .px4_qos import px4_input_qos, px4_output_qos
from .touchdown_detector import TouchdownDetector
from .visual_guidance import map_camera_error, VisualGuidance

VEHICLE_STATUS_TOPIC = '/fmu/out/vehicle_status_v1'
VEHICLE_LOCAL_POSITION_TOPIC = '/fmu/out/vehicle_local_position'
VEHICLE_ATTITUDE_TOPIC = '/fmu/out/vehicle_attitude'
VEHICLE_COMMAND_ACK_TOPIC = '/fmu/out/vehicle_command_ack'
VEHICLE_LAND_DETECTED_TOPIC = '/fmu/out/vehicle_land_detected'
OFFBOARD_CONTROL_MODE_TOPIC = '/fmu/in/offboard_control_mode'
TRAJECTORY_SETPOINT_TOPIC = '/fmu/in/trajectory_setpoint'
VEHICLE_COMMAND_TOPIC = '/fmu/in/vehicle_command'

TERMINAL_NO_OUTPUT_STATES = ('DATA_TIMEOUT', 'FAILSAFE', 'COMPLETE')
WAITING_STATES = ('WAIT_PX4', 'WAIT_SAFETY', 'WAIT_START')


class MissionControllerNode(Node):
    def __init__(self):
        super().__init__('mission_controller_node')
        defaults = {
            'mission_mode': 'hover_test', 'target_altitude': 1.0,
            'simulation_mode': False, 'competition_mode': False,
            'enable_control': False, 'enable_auto_arm': False,
            'enable_visual_follow': False, 'enable_payload_release': False,
            'enable_dynamic_landing': False, 'enable_auto_disarm': False,
            'enable_second_takeoff': False, 'prestream_cycles': 40,
            'allow_sitl_heading_quality_bypass': False,
            'hover_confirm_seconds': 3.0,
            'hover_test_seconds': 10.0, 'dwell_on_car_seconds': 5.0,
            'mission_timeout_seconds': 90.0, 'b_deadline_seconds': 15.0,
            'mission_deadline_s': 90.0, 'point_b_progress': -1,
            'point_d_progress': -1, 'return_reserve_s': 10.0,
            'progress_timeout_seconds': 2.0,
            'point_b_deadline_s': 15.0,
            'point_b_soft_deadline_s': 13.0,
            'point_d_deadline_s': 52.0,
            'payload_latest_command_s': 49.0,
            'altitude_tolerance': 0.1, 'stable_seconds': 1.0,
            'hover_altitude_m': 1.5, 'hover_altitude_tolerance_m': 0.1,
            'hover_max_vertical_speed_mps': 0.15,
            'status_timeout': 1.0, 'position_timeout': 0.5,
            'attitude_timeout': 0.5, 'safety_timeout': 1.0,
            'car_timeout': 2.0, 'max_tilt_rad': 0.45,
            'control_rate_hz': 20.0, 'align_error': 0.10,
            'max_horizontal_speed': 0.5,
            'high_descent_speed': 0.25, 'near_descent_speed': 0.10,
            'contact_descent_speed': 0.05, 'max_descent_speed': 0.4,
            'dynamic_near_height_m': 0.6, 'visual_stable_seconds': 0.5,
            'min_target_altitude': 0.5, 'max_target_altitude': 2.0,
            'command_timeout': 1.0, 'command_max_attempts': 3}
        defaults['odom_gate_min_frames'] = 20
        defaults['odom_gate_min_source_span_seconds'] = 1.0
        defaults['payload_ack_timeout'] = 5.0
        defaults['target_loss_abort_seconds'] = 1.0
        defaults['vision_timeout'] = 0.5
        defaults.update({
            'vision_tracked_topic': '/vision/target/tracked',
            'vision_landing_topic': '/vision/landing_error',
            'vision_min_confidence': 60.0,
            'vision_max_target_age_ms': 300.0,
            'vision_receive_timeout_sec': 0.3,
            'vision_swap_xy': False,
            'vision_x_sign': 1.0,
            'vision_y_sign': 1.0,
            'vision_target_x_norm': 0.0,
            'vision_target_y_norm': 0.0,
            'vision_kp_x': 0.25,
            'vision_kp_y': 0.25,
            'vision_max_speed_mps': 0.18,
            'vision_max_accel_mps2': 0.30,
            'vision_deadband_norm': 0.05,
            'align_tolerance_x_norm': 0.10,
            'align_tolerance_y_norm': 0.10,
            'align_stable_duration_sec': 0.8,
            'vision_loss_hover_sec': 0.20,
            'vision_loss_abort_sec': 1.0,
            'servo_command_topic': '/servo_command',
            'servo_result_topic': '/servo/result',
            'servo_release_timeout_sec': 5.0,
        })
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._validate_integration_parameters()
        logic_names = ('mission_mode', 'target_altitude', 'simulation_mode',
                       'competition_mode', 'enable_control', 'enable_auto_arm',
                       'enable_visual_follow', 'enable_payload_release',
                       'enable_dynamic_landing', 'enable_auto_disarm',
                       'enable_second_takeoff', 'prestream_cycles',
                       'hover_confirm_seconds',
                       'hover_test_seconds', 'dwell_on_car_seconds',
                       'mission_timeout_seconds', 'b_deadline_seconds',
                       'mission_deadline_s', 'point_b_progress',
                       'point_d_progress', 'return_reserve_s',
                       'progress_timeout_seconds',
                       'point_b_deadline_s', 'point_b_soft_deadline_s',
                       'point_d_deadline_s', 'payload_latest_command_s',
                       'altitude_tolerance', 'stable_seconds',
                       'hover_altitude_m', 'hover_altitude_tolerance_m',
                       'hover_max_vertical_speed_mps',
                       'visual_stable_seconds', 'dynamic_near_height_m')
        logic_names = logic_names + ('payload_ack_timeout',)
        logic_names = tuple(
            'visual_stable_seconds' if name == 'visual_stable_seconds' else name
            for name in logic_names)
        logic_values = {n: self.get_parameter(n).value for n in logic_names}
        logic_values['visual_stable_seconds'] = self.get_parameter(
            'align_stable_duration_sec').value
        logic_values['payload_ack_timeout'] = self.get_parameter(
            'servo_release_timeout_sec').value
        for name in ('point_b_progress', 'point_d_progress'):
            value = int(self.get_parameter(name).value)
            logic_values[name] = None if value < 0 else value
        if (self.get_parameter('mission_mode').value == 'drop' and
                (logic_values['point_b_progress'] is None or
                 logic_values['point_d_progress'] is None)):
            raise ValueError(
                'drop mode requires calibrated point_b_progress and '
                'point_d_progress; -1 means not configured')
        self.logic = MissionLogic(**logic_values)
        self.odom_gate = OdomFrameGate(
            self.get_parameter('odom_gate_min_frames').value,
            self.get_parameter(
                'odom_gate_min_source_span_seconds').value,
            self.get_parameter('position_timeout').value)
        if abs(float(self.get_parameter('control_rate_hz').value) - 20.0) > 1e-6:
            raise ValueError('formal mission control_rate_hz must be 20.0')
        if not prestream_is_safe(
                self.logic.enable_control, self.logic.prestream_cycles):
            raise ValueError('prestream_cycles must be at least 20 when flight is enabled')
        if not (self.get_parameter('min_target_altitude').value <=
                self.logic.target_altitude <=
                self.get_parameter('max_target_altitude').value):
            raise ValueError('target_altitude is outside configured safety limits')
        descent_names = ('high_descent_speed', 'near_descent_speed',
                         'contact_descent_speed')
        descent_values = [float(self.get_parameter(name).value)
                          for name in descent_names]
        max_descent = float(self.get_parameter('max_descent_speed').value)
        if (not all(math.isfinite(value) and 0.0 <= value <= max_descent
                    for value in descent_values) or
                not descent_values[0] >= descent_values[1] >=
                descent_values[2]):
            raise ValueError('descent speeds must be finite, limited and decreasing')
        self.guidance = VisualGuidance(
            kp_forward=self.get_parameter('vision_kp_y').value,
            kp_left=self.get_parameter('vision_kp_x').value,
            deadband_x=self.get_parameter('vision_deadband_norm').value,
            deadband_y=self.get_parameter('vision_deadband_norm').value,
            max_speed=self.get_parameter('vision_max_speed_mps').value,
            min_confidence=self.get_parameter('vision_min_confidence').value,
            max_age_ms=self.get_parameter('vision_max_target_age_ms').value,
            camera_x_sign=1.0,
            camera_y_sign=1.0,
            max_acceleration=self.get_parameter(
                'vision_max_accel_mps2').value)
        self.touchdown = TouchdownDetector()
        timeout = self.get_parameter('command_timeout').value
        attempts = self.get_parameter('command_max_attempts').value
        self.trackers = {name: CommandTracker(timeout, attempts) for name in
                         ('mode', 'arm', 'disarm', 'land')}
        self.last_status = self.last_position = self.last_attitude = None
        self.last_safety = self.last_car = None
        self.last_error = self.last_tracked = None
        self.landing_valid = self.tracked_valid = False
        self.roll = self.pitch = self.yaw = 0.0
        self.error = [0.0] * vision_schema.LANDING_ERROR_LENGTH
        self.tracked = vision_schema.invalid_tracked()
        self.touchdown_sensor = False
        self.px4_landed = False
        self.path = Path()
        self.path.header.frame_id = 'map_ned'
        self.last_state = None
        self.last_event = None
        self.payload_command_sent = False
        self.payload_request_monotonic = None
        self.payload_ack_seen = False
        self.payload_sequence_id = None
        self.next_payload_sequence_id = 0
        self.mapped_error = (0.0, 0.0)
        self.target_loss_since = None
        self.last_setpoint_mode = 'position'
        px4_sub_qos = px4_output_qos()
        px4_pub_qos = px4_input_qos()
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, OFFBOARD_CONTROL_MODE_TOPIC, px4_pub_qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, TRAJECTORY_SETPOINT_TOPIC, px4_pub_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, VEHICLE_COMMAND_TOPIC, px4_pub_qos)
        self.state_pub = self.create_publisher(String, '/uav/mission/state', 10)
        self.event_pub = self.create_publisher(String, '/uav/mission/event', 10)
        self.telemetry_pub = self.create_publisher(Float32MultiArray, '/uav/mission/telemetry', 10)
        self.path_pub = self.create_publisher(Path, '/uav/mission/path', 10)
        self.servo_command_pub = self.create_publisher(
            String, self.get_parameter('servo_command_topic').value, 10)
        self.create_subscription(
            VehicleStatus,
            VEHICLE_STATUS_TOPIC,
            self._status,
            px4_sub_qos)
        self.create_subscription(
            VehicleLocalPosition,
            VEHICLE_LOCAL_POSITION_TOPIC,
            self._position,
            px4_sub_qos)
        self.create_subscription(
            VehicleAttitude,
            VEHICLE_ATTITUDE_TOPIC,
            self._attitude,
            px4_sub_qos)
        self.create_subscription(
            VehicleCommandAck,
            VEHICLE_COMMAND_ACK_TOPIC,
            self._ack,
            px4_sub_qos)
        self.create_subscription(
            VehicleLandDetected,
            VEHICLE_LAND_DETECTED_TOPIC,
            self._land_detected,
            px4_sub_qos)
        self.create_subscription(Bool, '/car/mission_start', self._start, 10)
        self.create_subscription(
            String, '/uav_mission/events/start', self._start_event, 10)
        self.create_subscription(UInt8, '/car/progress', self._car, 10)
        self.create_subscription(Bool, '/uav/safety/ready', self._safety, 10)
        self.create_subscription(
            Bool, '/uav/mission/abort', self._abort, 10)
        self.create_subscription(Bool, '/uav/mission/reset', self._reset, 10)
        self.create_subscription(Bool, '/uav/touchdown_sensor', self._touchdown_sensor, 10)
        self.create_subscription(
            String, self.get_parameter('servo_result_topic').value,
            self._servo_result, 10)
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter('vision_landing_topic').value, self._error, 10)
        self.create_subscription(
            Float32MultiArray,
            self.get_parameter('vision_tracked_topic').value, self._tracked, 10)
        self.create_timer(1.0 / self.get_parameter('control_rate_hz').value, self._timer)

    def _validate_integration_parameters(self):
        """Reject unsafe visual and servo integration settings at startup."""
        names = (
            'vision_min_confidence', 'vision_max_target_age_ms',
            'vision_receive_timeout_sec', 'vision_kp_x', 'vision_kp_y',
            'vision_max_speed_mps', 'vision_max_accel_mps2',
            'vision_deadband_norm', 'align_tolerance_x_norm',
            'align_tolerance_y_norm', 'align_stable_duration_sec',
            'vision_loss_hover_sec', 'vision_loss_abort_sec',
            'servo_release_timeout_sec')
        values = [float(self.get_parameter(name).value) for name in names]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('visual and servo parameters must be finite')
        target_offsets = (
            float(self.get_parameter('vision_target_x_norm').value),
            float(self.get_parameter('vision_target_y_norm').value))
        if not all(math.isfinite(value) for value in target_offsets):
            raise ValueError('vision target offsets must be finite')
        if not 0.0 <= values[0] <= 100.0:
            raise ValueError('vision_min_confidence must be in [0, 100]')
        if values[1] <= 0.0 or values[2] <= 0.0:
            raise ValueError('vision age and receive timeout must be positive')
        if values[3] < 0.0 or values[4] < 0.0:
            raise ValueError('vision gains cannot be negative')
        if not 0.0 < values[5] <= 0.18:
            raise ValueError('vision_max_speed_mps must be in (0, 0.18]')
        if values[6] <= 0.0:
            raise ValueError('vision_max_accel_mps2 must be positive')
        if values[7] < 0.0 or values[8] < 0.0 or values[9] < 0.0:
            raise ValueError('visual deadband, tolerances and stable time cannot be negative')
        if values[10] < 0.0 or values[11] < 0.0 or values[12] <= 0.0:
            raise ValueError('vision loss times are invalid')
        if values[11] >= values[12]:
            raise ValueError('vision_loss_abort_sec must exceed hover time')
        if values[13] <= 0.0:
            raise ValueError('servo_release_timeout_sec must be positive')
        if (float(self.get_parameter('mission_deadline_s').value) <= 0.0 or
                float(self.get_parameter('return_reserve_s').value) < 0.0 or
                float(self.get_parameter('return_reserve_s').value) >=
                float(self.get_parameter('mission_deadline_s').value) or
                float(self.get_parameter('progress_timeout_seconds').value) <= 0.0):
            raise ValueError('mission timing parameters are invalid')
        timing = [float(self.get_parameter(name).value) for name in (
            'point_b_deadline_s', 'point_b_soft_deadline_s',
            'point_d_deadline_s', 'payload_latest_command_s')]
        if (not all(math.isfinite(value) for value in timing) or
                timing[1] < 0.0 or timing[0] <= timing[1] or
                timing[2] <= timing[0] or timing[3] < timing[0] or
                timing[3] >= timing[2] or
                timing[2] >= float(self.get_parameter(
                    'mission_deadline_s').value)):
            raise ValueError('point deadline parameters are invalid')
        for name in ('point_b_progress', 'point_d_progress'):
            point = int(self.get_parameter(name).value)
            if point != -1 and not 0 <= point <= 255:
                raise ValueError(f'{name} must be -1 or in [0, 255]')
        if (float(self.get_parameter('hover_altitude_m').value) <= 0.0 or
                float(self.get_parameter('hover_altitude_tolerance_m').value) < 0.0 or
                float(self.get_parameter('hover_max_vertical_speed_mps').value) <= 0.0):
            raise ValueError('hover altitude parameters are invalid')
        for name in ('vision_x_sign', 'vision_y_sign'):
            sign = float(self.get_parameter(name).value)
            if sign not in (-1.0, 1.0):
                raise ValueError(f'{name} must be -1 or 1')
        for name in ('vision_tracked_topic', 'vision_landing_topic',
                     'servo_command_topic', 'servo_result_topic'):
            if not str(self.get_parameter(name).value).strip():
                raise ValueError(f'{name} must not be empty')

    def _map_landing_error(self, error_x, error_y):
        """Map camera-normalized error into configurable controller axes."""
        return map_camera_error(
            error_x, error_y,
            self.get_parameter('vision_swap_xy').value,
            self.get_parameter('vision_x_sign').value,
            self.get_parameter('vision_y_sign').value,
            self.get_parameter('vision_target_x_norm').value,
            self.get_parameter('vision_target_y_norm').value)

    def now(self):
        return time.monotonic()

    def timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def _status(self, msg):
        received = self.now()
        self.last_status = received
        self.logic.update_status(msg.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                                 msg.nav_state,
                                 msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                                 msg.failsafe,
                                 received)

    def _position(self, msg):
        received = self.now()
        bypass = (self.logic.simulation_mode and
                  self.get_parameter('allow_sitl_heading_quality_bypass').value)
        values = (msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading)
        valid = (
            msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid and
            (msg.heading_good_for_control or bypass) and
            all(math.isfinite(value) for value in values))
        source_timestamp = (
            msg.timestamp_sample if msg.timestamp_sample != 0
            else msg.timestamp)
        new_frame = self.odom_gate.update(
            source_timestamp, received, valid)
        if new_frame:
            self.last_position = received
        self.logic.update_position(msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz, msg.heading, valid)
        self.logic.odom_ready = self.odom_gate.ready
        if valid and new_frame:
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
        if not quaternion_is_valid((w, x, y, z)):
            self.logic.attitude_valid = False
            self.logic.abnormal_tilt = True
            return
        self.roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        self.pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
        self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        self.logic.attitude_valid = all(math.isfinite(v)
                                        for v in (self.roll, self.pitch, self.yaw))
        self.logic.abnormal_tilt = max(abs(self.roll), abs(
            self.pitch)) > self.get_parameter('max_tilt_rad').value

    def _ack(self, msg):
        received = self.now()
        for tracker in self.trackers.values():
            if tracker.acknowledge(
                    msg.command, msg.result, received,
                    msg.target_system, msg.target_component,
                    getattr(self, 'command_context', ''),
                    msg.from_external):
                self.logic.event = 'COMMAND_ACK_%s_%s' % (msg.command, tracker.status)
                if tracker.failed:
                    self.logic.transition('FAILSAFE', self.now(), self.logic.event)
                break

    def _start(self, msg):
        self.logic.start_signal = bool(msg.data)

    def _start_event(self, msg):
        """Latch the gateway-confirmed first-start monotonic timestamp."""
        try:
            event = json.loads(msg.data)
            if event.get('type') != 'CAR_START' or not event.get('valid'):
                return
        except (TypeError, json.JSONDecodeError):
            return
        if self.logic.start_time_hint is None:
            self.logic.start_time_hint = time.monotonic()

    def _car(self, msg):
        self.last_car = self.now()
        self.logic.update_car_progress_at(msg.data, self.last_car)

    def _safety(self, msg):
        self.last_safety = self.now()
        self.logic.safety_ready = bool(msg.data)

    def _reset(self, msg):
        if msg.data:
            self.logic.reset_if_safe()

    def _abort(self, msg):
        if msg.data:
            self.logic.request_safe_abort(self.now())

    def _touchdown_sensor(self, msg): self.touchdown_sensor = bool(msg.data)

    def _land_detected(self, msg):
        self.px4_landed = bool(msg.landed)
        self.logic.update_landed(self.px4_landed)

    def _error(self, msg):
        self.last_error = self.now()
        try:
            self.error = vision_schema.validate_landing_error(msg.data)
            self.landing_valid = self.error[vision_schema.VALID] >= 0.5
        except ValueError:
            self.error = vision_schema.invalid_landing_error()
            self.landing_valid = False

    def _tracked(self, msg):
        self.last_tracked = self.now()
        try:
            self.tracked = vision_schema.validate_tracked(msg.data)
            self.tracked_valid = self.tracked[vision_schema.VALID] >= 0.5
        except ValueError:
            self.tracked = vision_schema.invalid_tracked()
            self.tracked_valid = False

    def _servo_result(self, msg):
        """Accept only a fresh exact ACK for the current servo request."""
        if (not self.payload_command_sent or
                self.payload_request_monotonic is None or
                self.logic.state != 'WAIT_RELEASE_ACK' or
                self.payload_ack_seen):
            return
        received = time.monotonic()
        if received <= self.payload_request_monotonic:
            return
        try:
            result = json.loads(str(msg.data))
            sequence_id = int(result['sequence_id'])
            status = str(result['status']).upper()
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            self.logic.event = 'SERVO_RESULT_IGNORED_MALFORMED'
            return
        if sequence_id != self.payload_sequence_id:
            self.logic.event = 'SERVO_RESULT_IGNORED_SEQUENCE'
            return
        if status == 'SUCCESS':
            self.payload_ack_seen = True
            self.logic.payload_ack = True
            self.logic.event = 'SERVO_SUCCESS_THROW'
        elif status in ('FAILED', 'REJECTED', 'DUPLICATE'):
            self.payload_ack_seen = True
            self.logic.payload_failure = status
            self.logic.safety_block = 'PAYLOAD_ACK_FAILED'
            self.logic.transition('FAILSAFE', self.now(),
                                  'SERVO_%s' % status)
        else:
            self.logic.event = 'SERVO_RESULT_IGNORED'

    def _publish_command(self, name, command, now, param1=0.0, param2=0.0):
        tracker = self.trackers[name]
        if not tracker.request(
                command, now, getattr(self, 'command_context', ''), 1, 1):
            if tracker.failed:
                self.logic.transition(
                    'FAILSAFE', now, 'COMMAND_%s_%s' %
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
        self.command_pub.publish(msg)

    def _publish_control(self, mode, position=None, velocity=None):
        if mode in ('position', 'position_velocity'):
            if position is None or not all(math.isfinite(float(value))
                                           for value in position):
                self.logic.event = 'CONTROL_SETPOINT_REJECTED_NONFINITE'
                return
        if velocity is not None:
            velocity = tuple(float(value) for value in velocity)
            if not all(math.isfinite(value) for value in velocity):
                self.logic.event = 'CONTROL_SETPOINT_REJECTED_NONFINITE'
                return
            horizontal_speed = math.hypot(velocity[0], velocity[1])
            maximum_speed = self.get_parameter('vision_max_speed_mps').value
            if horizontal_speed > maximum_speed:
                scale = maximum_speed / horizontal_speed
                velocity = (velocity[0] * scale, velocity[1] * scale,
                            velocity[2])
        heartbeat = OffboardControlMode()
        heartbeat.timestamp = self.timestamp()
        heartbeat.position = mode in ('position', 'position_velocity')
        heartbeat.velocity = mode in ('velocity', 'position_velocity')
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
        self.odom_gate.expire(now)
        self.logic.odom_ready = self.odom_gate.ready
        position_invalid = not self.logic.px4_fresh
        attitude_stale = (self.last_attitude is None or
                          now - self.last_attitude > attitude_timeout)
        if status_stale or position_stale or position_invalid:
            self.logic.px4_fresh = False
        if attitude_stale:
            self.logic.attitude_valid = False
        px4_stale = (
            status_stale or position_stale or position_invalid or
            (self.logic.state not in WAITING_STATES and
             not self.logic.odom_ready) or attitude_stale)
        if px4_stale and self.logic.state not in (
                WAITING_STATES + TERMINAL_NO_OUTPUT_STATES +
                ('FINAL_LAND',)):
            self.logic.safety_block = 'PX4_DATA_TIMEOUT'
            action = data_loss_action(self.logic.armed, status_stale)
            event = ('PX4_DATA_TIMEOUT_LAND' if action == 'FINAL_LAND'
                     else 'PX4_DATA_TIMEOUT')
            self.logic.transition(action, now, event)
        if (px4_stale and self.logic.state == 'FINAL_LAND' and
                self.logic.armed and not status_stale):
            # Position/attitude loss in flight must first hand control to
            # PX4's own landing mode. Do not silently stop Offboard output.
            self._publish_command(
                'land', VehicleCommand.VEHICLE_CMD_NAV_LAND, now)
            self._publish_observability(now)
            self.last_state = self.logic.state
            return
        if self.logic.state == 'DATA_TIMEOUT':
            self._publish_observability(now)
            self.last_state = self.logic.state
            return
        if not self.logic.simulation_mode and (
                self.last_safety is None or now -
                self.last_safety > self.get_parameter('safety_timeout').value):
            self.logic.safety_ready = False
        vision_receive_timeout = self.get_parameter(
            'vision_receive_timeout_sec').value
        vision_max_age = self.get_parameter('vision_max_target_age_ms').value
        landing_fresh = vision_is_fresh(
            now, self.last_error, self.error[vision_schema.ERROR_TARGET_AGE_MS],
            vision_receive_timeout, vision_max_age)
        tracked_fresh = vision_is_fresh(
            now, self.last_tracked,
            self.tracked[vision_schema.TRACKED_TARGET_AGE_MS],
            vision_receive_timeout, vision_max_age)
        landing_valid = (self.landing_valid and
                         self.error[vision_schema.VALID] >= 0.5)
        tracked_valid = (self.tracked_valid and
                         self.tracked[vision_schema.VALID] >= 0.5)
        confidence = self.error[vision_schema.ERROR_CONFIDENCE]
        age = self.error[vision_schema.ERROR_TARGET_AGE_MS]
        ex, ey = self._map_landing_error(
            self.error[vision_schema.ERROR_X_NORMALIZED],
            self.error[vision_schema.ERROR_Y_NORMALIZED])
        self.mapped_error = (ex, ey)
        vision_values_finite = all(math.isfinite(value)
                                   for value in (confidence, age, ex, ey))
        target_confidence = min(
            confidence, self.tracked[vision_schema.TRACKED_CONFIDENCE])
        target_ok = (landing_fresh and tracked_fresh and landing_valid and
                     tracked_valid and vision_values_finite and
                     target_confidence >= self.get_parameter(
                         'vision_min_confidence').value)
        aligned = (target_ok and
                   abs(ex) <= self.get_parameter(
                       'align_tolerance_x_norm').value and
                   abs(ey) <= self.get_parameter(
                       'align_tolerance_y_norm').value)
        self.logic.update_visual(target_ok, aligned, now)
        visual_states = ('VISION_FOLLOW', 'ALIGN_FOR_DROP', 'ALIGN_PLATFORM',
                         'DYNAMIC_DESCENT_HIGH', 'DYNAMIC_DESCENT_NEAR',
                         'TOUCHDOWN_CHECK')
        if self.logic.state in visual_states and not target_ok:
            if self.target_loss_since is None:
                self.target_loss_since = now
            loss_duration = now - self.target_loss_since
            if loss_duration >= self.get_parameter(
                    'vision_loss_abort_sec').value:
                self.logic.safety_block = 'VISION_INVALID'
                target_state = ('RETURN_HOME' if self.logic.h is not None and
                                self.logic.position is not None else 'FAILSAFE')
                self.logic.transition(target_state, now, 'VISION_LOSS_ABORT')
            elif (loss_duration >= self.get_parameter(
                    'vision_loss_hover_sec').value and
                  self.logic.state in ('VISION_FOLLOW', 'ALIGN_FOR_DROP',
                                       'ALIGN_PLATFORM')):
                self.logic.transition('SEARCH_CAR', now, 'VISION_LOSS_SEARCH')
        elif target_ok:
            self.target_loss_since = None
        descent_states = (
            'DYNAMIC_DESCENT_HIGH', 'DYNAMIC_DESCENT_NEAR',
            'TOUCHDOWN_CHECK')
        if (self.logic.state in descent_states and
                (self.last_car is None or
                 now - self.last_car > self.get_parameter('car_timeout').value)):
            self.logic.transition('RETURN_HOME', now, 'CAR_DATA_TIMEOUT')
        kinematic = (self.logic.simulation_mode and
                     self.logic.state in descent_states and
                     self.logic.position is not None and
                     self.logic.position[2] >= self.logic.h[2] - 0.12)
        self.logic.touchdown_candidate = self.touchdown.candidate(
            self.touchdown_sensor, kinematic, self.logic.velocity[2],
            self.roll, self.pitch)
        touchdown_confirmed_now = self.touchdown.update(
            now,
            self.touchdown_sensor,
            kinematic,
            self.logic.velocity[2],
            self.roll,
            self.pitch)
        if touchdown_confirmed_now:
            self.logic.touchdown = True
        elif self.logic.state in descent_states:
            # Confirmation must be continuous while approaching. Once the
            # state machine accepts touchdown, latch it through the dwell and
            # disarm transaction instead of clearing it on the next cycle.
            self.logic.touchdown = False
        self.logic.step(now)
        if self.logic.state == 'SECOND_PRESTREAM' and self.last_state != self.logic.state:
            self.trackers['mode'].reset()
            self.trackers['arm'].reset()
            self.trackers['land'].reset()
        if (self.logic.enable_control and
                control_output_allowed(self.logic.state, px4_stale) and
                self.logic.state not in (
                    'WAIT_PX4',
                    'WAIT_SAFETY',
                    'WAIT_START',
                    'COMPLETE',
                    'FAILSAFE')):
            if self.logic.state == 'REQUEST_OFFBOARD':
                if vehicle_command_allowed(
                        self.logic.enable_control, self.logic.state, px4_stale):
                    self._publish_command(
                        'mode', VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                        now, 1.0, 6.0)
            if (self.logic.state in ('ARMING', 'SECOND_ARM') and
                    self.logic.auto_arm_allowed()):
                self._publish_command(
                    'arm', VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, now, 1.0)
            if (self.logic.state == 'DISARM_ON_CAR' and
                    self.logic.target_ok):
                if self.logic.simulation_mode and not self.px4_landed:
                    # Hand control to PX4 without invoking Offboard-loss
                    # failsafe, then wait for PX4's own landed signal.
                    self._publish_command(
                        'land', VehicleCommand.VEHICLE_CMD_NAV_LAND, now)
                elif self.logic.auto_disarm_allowed(self.touchdown_sensor):
                    self._publish_command(
                        'disarm',
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        now, 0.0)
            if (self.logic.state == 'FINAL_LAND' and self.logic.armed and
                    self.logic.nav_state !=
                    VehicleStatus.NAVIGATION_STATE_AUTO_LAND):
                self._publish_command(
                    'land', VehicleCommand.VEHICLE_CMD_NAV_LAND, now)
            if self.logic.state in (
                    'VISION_FOLLOW', 'ALIGN_FOR_DROP', 'ALIGN_PLATFORM',
                    'DYNAMIC_DESCENT_HIGH', 'DYNAMIC_DESCENT_NEAR',
                    'TOUCHDOWN_CHECK'):
                if not self.logic.target_ok:
                    n, e = self.guidance.velocity(
                        False, ex, ey, confidence, age, self.logic.heading,
                        dt=1.0 / self.get_parameter('control_rate_hz').value)
                    self._publish_control('velocity', velocity=(n, e, 0.0))
                    self.logic.record_prestream_cycle()
                    self._publish_observability(now)
                    self.last_state = self.logic.state
                    return
                n, e = self.guidance.velocity(
                    target_ok, ex, ey, target_confidence, age,
                    self.logic.heading,
                    dt=1.0 / self.get_parameter('control_rate_hz').value)
                down = 0.0
                if self.logic.visual_stable(now) and self.logic.aligned:
                    down = descent_speed_for_state(
                        self.logic.state,
                        self.get_parameter('high_descent_speed').value,
                        self.get_parameter('near_descent_speed').value,
                        self.get_parameter('contact_descent_speed').value)
                self._publish_control('velocity', velocity=(n, e, down))
            elif (self.logic.state in (
                    'LANDED_ON_CAR', 'DWELL_5S') and
                    self.logic.position is not None):
                if self.logic.simulation_mode:
                    # The SITL kinematic detector can confirm slightly above
                    # the collision surface. Keep a small downward contact
                    # command so PX4's independent land detector can settle
                    # before it is asked to disarm.
                    self._publish_control(
                        'velocity',
                        velocity=(0.0, 0.0,
                                  self.get_parameter(
                                      'contact_descent_speed').value))
                else:
                    self._publish_control(
                        'position', position=self.logic.position)
            elif self.logic.state == 'FINAL_LAND':
                # NAV_LAND transfers control back to PX4. Continuing to send
                # the cruise-altitude Offboard setpoint here can keep the
                # land detector from settling and delay PX4's own disarm.
                pass
            elif self.logic.state == 'DISARM_ON_CAR':
                # Stop Offboard output before asking PX4 to disarm. In SITL
                # this lets PX4's own collision/land detector settle; on
                # hardware the physical-touchdown gate is still mandatory.
                pass
            elif self.logic.h is not None:
                self._publish_control(
                    'position',
                    position=(
                        self.logic.h[0],
                        self.logic.h[1],
                        self.logic.cruise_z))
            self.logic.record_prestream_cycle()
        if (self.logic.payload_sent and
                not self.payload_command_sent and
                self.get_parameter('enable_payload_release').value):
            self.logic.payload_ack = False
            self.logic.payload_failure = None
            self.payload_ack_seen = False
            self.payload_request_monotonic = time.monotonic()
            self.next_payload_sequence_id = max(
                self.next_payload_sequence_id + 1, time.time_ns())
            self.payload_sequence_id = self.next_payload_sequence_id
            self.payload_command_sent = True
            self.logic.transition(
                'WAIT_RELEASE_ACK', now, 'SERVO_THROW_REQUESTED')
            command = String()
            command.data = json.dumps({
                'command': 'throw',
                'sequence_id': self.payload_sequence_id,
                'mission_started_at': self.logic.started_at,
            }, separators=(',', ':'))
            self.servo_command_pub.publish(command)
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
        state_elapsed = max(0.0, now - self.logic.state_since)
        vision_age_ms = (-1.0 if self.last_error is None else
                         max(0.0, (now - self.last_error) * 1000.0))
        values = {'mission_active': self.logic.state not in ('WAIT_PX4',
                                                             'WAIT_SAFETY',
                                                             'WAIT_START',
                                                             'DATA_TIMEOUT',
                                                             'FAILSAFE',
                                                             'COMPLETE'),
                  'mission_mode': int(self.logic.mode),
                  'state_id': STATE_ID[self.logic.state],
                  'elapsed_seconds': elapsed,
                  'state_elapsed_seconds': state_elapsed,
                  'remaining_seconds': self.logic.remaining_seconds(now),
                  'heading': self.logic.heading,
                  'target_valid': self.logic.target_ok,
                  'vision_age_ms': vision_age_ms,
                  'target_error_x': self.mapped_error[0],
                  'target_error_y': self.mapped_error[1],
                  'target_confidence': min(
                      self.error[6], self.tracked[vision_schema.TRACKED_CONFIDENCE]),
                  'car_progress': int(self.logic.car_progress),
                  'formed_follow_before_b': self.logic.formed_follow_before_b,
                  'completed_before_d': self.logic.completed_before_d,
                  'armed': self.logic.armed,
                  'nav_state': self.logic.nav_state,
                  'offboard_active': self.logic.offboard,
                  'failsafe': self.logic.failsafe,
                  'px4_fresh': self.logic.px4_fresh and self.logic.attitude_valid,
                  'payload_sent': self.logic.payload_sent,
                  'payload_ack': self.logic.payload_ack,
                  'command_ack_status': max(t.attempts for t in self.trackers.values()),
                  'touchdown_candidate': self.logic.touchdown_candidate,
                  'touchdown_confirmed': self.logic.touchdown,
                  'dwell_progress': self.logic.dwell_progress(now),
                  'safety_block_code': SAFETY_BLOCK_CODES.get(
                      self.logic.safety_block, 0)}
        if self.logic.position:
            for key, value in zip(('x', 'y', 'z', 'vx', 'vy', 'vz'),
                                  self.logic.position + self.logic.velocity):
                values[key] = value
            if self.logic.h:
                values['h_distance'] = math.hypot(
                    self.logic.position[0] - self.logic.h[0],
                    self.logic.position[1] - self.logic.h[1])
                values['relative_h_height'] = self.logic.relative_h_height
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
