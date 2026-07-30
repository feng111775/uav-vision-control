"""Stage 4C high-level adapter around the accepted 4B PX4 controller."""

import json
import math

from px4_msgs.msg import VehicleCommand
import rclpy
from std_msgs.msg import String

from .mission_controller_node import MissionControllerNode


class MissionOffboardController(MissionControllerNode):
    """The only stage-4C node that owns PX4 input publishers."""

    def __init__(self):
        super().__init__()
        self.high_level_command = {'mode': 'HOLD', 'target': None}
        self.stage4c_prestream_cycles = 0
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
        if command.get('mode') not in ('HOLD', 'POSITION', 'VISION', 'LAND'):
            self.get_logger().warning('unsupported high-level mode ignored')
            return
        self.high_level_command = command

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
        if mode == 'POSITION' and self._valid_target(target):
            self._publish_control('position', position=target)
        elif mode == 'VISION' and self._valid_target(target):
            north, east = self.guidance.velocity(
                True, float(target[0]), float(target[1]), 100.0, 0.0,
                self.logic.heading)
            self._publish_control('velocity', velocity=(north, east, 0.0))
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
        if (phase == 'TAKEOFF' and not self.logic.armed and
                self.logic.enable_auto_arm):
            self._publish_command(
                'arm', VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                now, 1.0)

    @staticmethod
    def _valid_target(target):
        """Require a finite three-axis high-level target."""
        return (isinstance(target, (list, tuple)) and len(target) == 3 and
                all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in target))

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
