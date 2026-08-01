"""Stage 4C high-level adapter around the accepted 4B PX4 controller."""

import json
import math

from px4_msgs.msg import VehicleCommand
import rclpy
from std_msgs.msg import String

from .mission_controller_node import MissionControllerNode
from .stage4c_core import (
    HorizontalVelocityLimiter, resolve_takeoff_height,
    validate_control_interlocks)
from .visual_guidance import VisualGuidance


class MissionOffboardController(MissionControllerNode):
    """The only stage-4C node that owns PX4 input publishers."""

    def __init__(self):
        super().__init__()
        self.declare_parameter('max_vertical_speed', 0.3)
        self.declare_parameter('search_speed_mps', 0.18)
        self.declare_parameter('search_max_speed_mps', 0.18)
        self.declare_parameter('search_acceleration_mps2', 0.1)
        self.declare_parameter('follow_kp_forward', 0.3)
        self.declare_parameter('follow_kp_left', 0.3)
        self.declare_parameter('follow_deadband_x', 0.04)
        self.declare_parameter('follow_deadband_y', 0.04)
        self.declare_parameter('follow_max_speed_mps', 0.2)
        self.declare_parameter('follow_min_confidence', 0.6)
        self.declare_parameter('follow_swap_axes', False)
        self.declare_parameter('follow_error_scale', 1.0)
        self.declare_parameter('follow_camera_x_sign', 1.0)
        self.declare_parameter('follow_camera_y_sign', -1.0)
        self.declare_parameter('follow_camera_mount_yaw_rad', 0.0)
        self.declare_parameter('return_position_kp', 0.5)
        self.declare_parameter('return_max_speed_mps', 0.4)
        self.declare_parameter('return_acceleration_mps2', 0.2)
        self.declare_parameter('confirm_sitl_only', False)
        self.declare_parameter('hardware_bench_mode', False)
        self.declare_parameter('takeoff_height_m', 1.5)
        self.declare_parameter('physical_release_enabled', False)
        simulation = bool(self.logic.simulation_mode)
        confirm_sitl = bool(
            self.get_parameter('confirm_sitl_only').value)
        bench = bool(self.get_parameter('hardware_bench_mode').value)
        validate_control_interlocks(
            simulation, confirm_sitl, self.logic.enable_control,
            self.logic.enable_auto_arm, bench)
        if self.get_parameter('physical_release_enabled').value:
            raise ValueError('physical release is unavailable in stage 4C-3A')
        resolve_takeoff_height(
            self.get_parameter('takeoff_height_m').value,
            self.logic.target_altitude)
        search_speed = float(self.get_parameter('search_speed_mps').value)
        search_limit = float(
            self.get_parameter('search_max_speed_mps').value)
        search_acceleration = float(
            self.get_parameter('search_acceleration_mps2').value)
        if not all(math.isfinite(value) for value in (
                search_speed, search_limit, search_acceleration)):
            raise ValueError('search motion parameters must be finite')
        if not 0.0 <= search_speed <= search_limit <= 0.5:
            raise ValueError('search speed exceeds safe range')
        if search_acceleration <= 0.0:
            raise ValueError('search acceleration must be positive')
        self.get_logger().warning(
            'target_altitude is deprecated; takeoff_height_m is authoritative')
        self.get_logger().warning(
            'mode=%s control=%s auto_arm=%s physical_release=false' % (
                'HARDWARE_BENCH' if bench else
                ('SITL' if simulation else 'HARDWARE_MONITOR'),
                self.logic.enable_control, self.logic.enable_auto_arm))
        self.high_level_command = {'mode': 'HOLD', 'target': None}
        self.command_context = ''
        self.stage4c_prestream_cycles = 0
        self.stage4c_position_setpoint = None
        self.velocity_limiter = HorizontalVelocityLimiter(
            search_acceleration, search_limit)
        self.return_velocity_limiter = HorizontalVelocityLimiter(
            self.get_parameter('return_acceleration_mps2').value,
            self.get_parameter('return_max_speed_mps').value)
        return_kp = float(self.get_parameter('return_position_kp').value)
        if not math.isfinite(return_kp) or return_kp <= 0.0:
            raise ValueError('return_position_kp must be finite and positive')
        self.return_position_kp = return_kp
        self.guidance = VisualGuidance(
            kp_forward=self.get_parameter('follow_kp_forward').value,
            kp_left=self.get_parameter('follow_kp_left').value,
            deadband_x=self.get_parameter('follow_deadband_x').value,
            deadband_y=self.get_parameter('follow_deadband_y').value,
            max_speed=self.get_parameter('follow_max_speed_mps').value,
            min_confidence=100.0 * self.get_parameter(
                'follow_min_confidence').value,
            camera_x_sign=self.get_parameter('follow_camera_x_sign').value,
            camera_y_sign=self.get_parameter('follow_camera_y_sign').value,
            swap_axes=self.get_parameter('follow_swap_axes').value,
            error_scale=self.get_parameter('follow_error_scale').value,
            camera_mount_yaw_rad=self.get_parameter(
                'follow_camera_mount_yaw_rad').value)
        self.last_search_log = None
        self.command_subscription = self.create_subscription(
            String, '/uav_mission/control/command', self._high_level, 10)
        self.stage4c_status = self.create_publisher(
            String, '/uav_mission/control/status', 10)
        self.create_timer(0.1, self._stage4c_status)

    def _high_level(self, msg):
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning('invalid high-level command ignored')
            return
        if command.get('mode') not in (
                'HOLD', 'POSITION', 'SEARCH', 'BRAKE', 'VISION', 'LAND'):
            self.get_logger().warning('unsupported high-level mode ignored')
            return
        self.high_level_command = command
        task_id = str(command.get('task_id', ''))
        if task_id != self.command_context:
            self.command_context = task_id
            for tracker in self.trackers.values():
                tracker.reset()

    def _fresh_and_safe(self, now):
        """Apply the accepted 4B freshness and failsafe gates."""
        status_age = (
            math.inf if self.last_status is None else now - self.last_status)
        position_age = (
            math.inf if self.last_position is None else now - self.last_position)
        attitude_age = (
            math.inf if self.last_attitude is None else now - self.last_attitude)
        return (
            status_age <= self.get_parameter('status_timeout').value and
            position_age <= self.get_parameter('position_timeout').value and
            attitude_age <= self.get_parameter('attitude_timeout').value and
            self.logic.px4_fresh and self.logic.odom_ready and
            self.logic.attitude_valid and not self.logic.failsafe)

    def _timer(self):
        """Translate manager commands while retaining 4B safety primitives."""
        now = self.now()
        if not self.logic.enable_control:
            return
        if not self._fresh_and_safe(now):
            if self.logic.armed and self.last_status is not None:
                self._publish_command(
                    'land', VehicleCommand.VEHICLE_CMD_NAV_LAND, now)
            return
        if not self.logic.simulation_mode and not self.logic.safety_ready:
            return
        mode = self.high_level_command.get('mode', 'HOLD')
        phase = self.high_level_command.get('phase', 'WAIT_FOR_START')
        target = self.high_level_command.get('target')
        if mode == 'LAND':
            self._publish_command(
                'land', VehicleCommand.VEHICLE_CMD_NAV_LAND, now)
            return
        if phase in ('WAIT_FOR_START', 'INITIALIZING', 'COMPLETE'):
            return
        if (mode == 'POSITION' and
                phase in ('RETURN_HOME', 'ABORT_RETURN') and
                self._valid_target(target) and self.logic.position is not None):
            error = (
                float(target[0]) - self.logic.position[0],
                float(target[1]) - self.logic.position[1],
            )
            velocity = self.return_velocity_limiter.update(
                (self.return_position_kp * error[0],
                 self.return_position_kp * error[1]),
                1.0 / float(self.get_parameter('control_rate_hz').value))
            self._publish_control(
                'position_velocity',
                position=(float('nan'), float('nan'), float(target[2])),
                velocity=(velocity[0], velocity[1], float('nan')))
        elif mode == 'POSITION' and self._valid_target(target):
            self._publish_control(
                'position', position=self._bounded_position_target(target))
        elif mode == 'SEARCH' and self._valid_search_command(
                self.high_level_command):
            velocity = self._bounded_velocity_target(
                self.high_level_command['velocity'])
            nan = float('nan')
            self._publish_control(
                'position_velocity',
                position=(nan, nan, self.high_level_command['target_z']),
                velocity=(velocity[0], velocity[1], nan))
            self._log_search(now, velocity)
        elif mode == 'VISION' and self._valid_target(target):
            observation_age_ms = max(
                0.0, (now - float(self.high_level_command.get(
                    'observation_stamp', 0.0))) * 1000.0)
            north, east = self.guidance.velocity(
                True, float(target[0]), float(target[1]),
                100.0 * float(self.high_level_command.get(
                    'confidence', 0.0)),
                observation_age_ms,
                self.logic.heading)
            velocity = self._bounded_velocity_target((north, east))
            self._publish_control(
                'position_velocity',
                position=(float('nan'), float('nan'), float(target[2])),
                velocity=(velocity[0], velocity[1], float('nan')))
        elif mode == 'BRAKE' and isinstance(
                self.high_level_command.get('target_z'), (int, float)):
            velocity = self._bounded_velocity_target((0.0, 0.0))
            self._publish_control(
                'position_velocity',
                position=(float('nan'), float('nan'),
                          float(self.high_level_command['target_z'])),
                velocity=(velocity[0], velocity[1], float('nan')))
        elif mode == 'HOLD' and isinstance(
                self.high_level_command.get('target_z'), (int, float)):
            velocity = self._bounded_velocity_target((0.0, 0.0))
            self._publish_control(
                'position_velocity',
                position=(float('nan'), float('nan'),
                          float(self.high_level_command['target_z'])),
                velocity=(velocity[0], velocity[1], float('nan')))
        elif self.logic.position is not None:
            self._publish_control('position', position=self.logic.position)
        else:
            return
        self.stage4c_prestream_cycles += 1
        if self.stage4c_prestream_cycles < self.logic.prestream_cycles:
            return
        if not self.logic.offboard:
            self._publish_command(
                'mode', VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                now, 1.0, 6.0)
        if (phase == 'TAKEOFF' and self.logic.offboard and
                not self.logic.armed and self.logic.enable_auto_arm):
            self._publish_command(
                'arm', VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                now, 1.0)

    @staticmethod
    def _valid_target(target):
        """Require a finite three-axis high-level target."""
        return (isinstance(target, (list, tuple)) and len(target) == 3 and
                all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in target))

    def _bounded_position_target(self, target):
        """Rate-limit a position target using configured axis speeds."""
        if self.logic.position is None:
            return target
        if self.stage4c_position_setpoint is None:
            self.stage4c_position_setpoint = tuple(self.logic.position)
        rate = float(self.get_parameter('control_rate_hz').value)
        horizontal_step = (
            float(self.get_parameter('max_horizontal_speed').value) / rate)
        vertical_step = (
            float(self.get_parameter('max_vertical_speed').value) / rate)
        current = self.stage4c_position_setpoint
        dx = float(target[0]) - current[0]
        dy = float(target[1]) - current[1]
        distance = math.hypot(dx, dy)
        scale = 1.0 if distance <= horizontal_step else \
            horizontal_step / distance
        dz = max(-vertical_step, min(
            vertical_step, float(target[2]) - current[2]))
        self.stage4c_position_setpoint = (
            current[0] + dx * scale,
            current[1] + dy * scale,
            current[2] + dz,
        )
        return self.stage4c_position_setpoint

    @staticmethod
    def _valid_search_command(command):
        velocity = command.get('velocity')
        target_z = command.get('target_z')
        return (
            isinstance(velocity, (list, tuple)) and len(velocity) == 2 and
            all(isinstance(value, (int, float)) and math.isfinite(value)
                for value in velocity) and
            isinstance(target_z, (int, float)) and math.isfinite(target_z))

    def _bounded_velocity_target(self, target):
        """Slew-limit horizontal velocity across SEARCH/FOLLOW transitions."""
        rate = float(self.get_parameter('control_rate_hz').value)
        return self.velocity_limiter.update(target, 1.0 / rate)

    def _log_search(self, now, velocity):
        if self.last_search_log is None or now - self.last_search_log >= 1.0:
            self.last_search_log = now
            self.get_logger().info(
                'SEARCH_CAR heading=%s vx=%.3f vy=%.3f speed=%.3f' % (
                    self.high_level_command.get('search_heading'),
                    velocity[0], velocity[1], math.hypot(*velocity)))

    def _stage4c_status(self):
        msg = String()
        msg.data = json.dumps({
            'stamp_ns': self.get_clock().now().nanoseconds,
            'accepted_mode': self.high_level_command['mode'],
            'px4_fresh': bool(self.logic.px4_fresh),
            'failsafe': bool(self.logic.failsafe),
            'home_recorded': self.logic.h is not None,
            'legacy_safety_state': self.logic.state,
        }, separators=(',', ':'))
        self.stage4c_status.publish(msg)


def main(args=None):
    """Run the sole stage-4C PX4 control owner."""
    rclpy.init(args=args)
    node = MissionOffboardController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
