"""将任务地图轨迹转换为PX4 NED位置设定值的ROS 2控制节点。"""

import math
from typing import Optional

from mission_interfaces.msg import MissionStatus
from mission_interfaces.msg import TrajectoryPoint as MissionTrajectoryPoint
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleCommandAck
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from .coordinate_transform import CoordinateTransformer, PX4Coordinate


class MissionOffboardLogic:
    """保存任务输入状态并判断是否允许发送PX4位置目标。"""

    ACTIVE_STATES = (
        MissionStatus.TAKEOFF,
        MissionStatus.EXECUTE,
        MissionStatus.RETURN,
    )

    def __init__(
            self,
            transformer: CoordinateTransformer,
            target_timeout: float = 0.5,
            mission_status_timeout: float = 0.5,
            local_position_timeout: float = 0.5,
    ) -> None:
        """设置坐标转换器和输入消息超时限制。"""
        timeout_values = (
            target_timeout,
            mission_status_timeout,
            local_position_timeout,
        )
        if not all(
                math.isfinite(float(value)) and float(value) > 0.0
                for value in timeout_values):
            raise ValueError('输入消息超时参数必须是有限正数')

        self.transformer = transformer
        self.target_timeout = float(target_timeout)
        self.mission_status_timeout = float(mission_status_timeout)
        self.local_position_timeout = float(local_position_timeout)
        self.mission_state = MissionStatus.IDLE
        self.mission_status_time: Optional[float] = None
        self.target: Optional[PX4Coordinate] = None
        self.target_yaw = math.nan
        self.target_time: Optional[float] = None
        self.local_position_valid = False
        self.local_position_time: Optional[float] = None

    def update_mission_status(self, state: int, now_seconds: float) -> None:
        """保存最新任务状态及其本地接收时间。"""
        self.mission_state = int(state)
        self.mission_status_time = float(now_seconds)

    def update_target(
            self,
            x: float,
            y: float,
            z: float,
            yaw: float,
            now_seconds: float,
    ) -> None:
        """将地图目标转换为PX4 NED目标并保存接收时间。"""
        self.target = self.transformer.map_to_px4_position(x, y, z)
        self.target_yaw = self.transformer.map_to_px4_yaw(yaw)
        self.target_time = float(now_seconds)

    def update_local_position(
            self,
            x: float,
            y: float,
            z: float,
            xy_valid: bool,
            z_valid: bool,
            now_seconds: float,
    ) -> None:
        """保存PX4本地位置有效性和接收时间。"""
        coordinates_finite = all(
            math.isfinite(float(value)) for value in (x, y, z))
        self.local_position_valid = (
            bool(xy_valid) and bool(z_valid) and coordinates_finite)
        self.local_position_time = float(now_seconds)

    @staticmethod
    def _fresh(
            now_seconds: float,
            message_time: Optional[float],
            timeout: float,
    ) -> bool:
        """判断输入是否存在、未来自未来且没有超时。"""
        if message_time is None:
            return False
        age = float(now_seconds) - message_time
        return 0.0 <= age <= timeout

    def target_allowed(self, now_seconds: float) -> bool:
        """仅在任务活跃且所有关键输入有效、新鲜时允许发送目标。"""
        return all((
            self.mission_state in self.ACTIVE_STATES,
            self.target is not None,
            self.local_position_valid,
            self._fresh(
                now_seconds, self.mission_status_time,
                self.mission_status_timeout),
            self._fresh(
                now_seconds, self.target_time, self.target_timeout),
            self._fresh(
                now_seconds, self.local_position_time,
                self.local_position_timeout),
        ))


class MissionOffboardController(Node):
    """以10 Hz向PX4发布经过安全过滤的任务位置设定值。"""

    TIMER_PERIOD = 0.1
    PRESTREAM_DURATION = 2.0
    HORIZONTAL_SPEED_THRESHOLD = 0.1
    HORIZONTAL_STABILITY_DURATION = 1.0
    GROUND_Z_THRESHOLD = -0.2
    GROUND_DISARM_DURATION = 2.0
    COMMAND_RETRY_INTERVAL = 1.0

    WAIT = 'WAIT'
    PRESTREAM = 'PRESTREAM'
    REQUEST_OFFBOARD = 'REQUEST_OFFBOARD'
    REQUEST_ARM = 'REQUEST_ARM'
    TAKEOFF_HOLD = 'TAKEOFF_HOLD'
    MISSION_EXECUTE = 'MISSION_EXECUTE'

    def __init__(self) -> None:
        """创建参数、任务接口、PX4接口和控制定时器。"""
        super().__init__('mission_offboard_controller')

        self.declare_parameter('origin_north', 15.0)
        self.declare_parameter('origin_east', -20.0)
        self.declare_parameter('origin_down', 0.0)
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('target_timeout', 0.5)
        self.declare_parameter('mission_status_timeout', 0.5)
        self.declare_parameter('local_position_timeout', 0.5)
        self.declare_parameter('takeoff_height', 2.0)
        self.declare_parameter('takeoff_hold_time', 5.0)

        self.takeoff_height = float(
            self.get_parameter('takeoff_height').value)
        if (
                not math.isfinite(self.takeoff_height)
                or self.takeoff_height <= 0.0):
            raise ValueError('takeoff_height必须是有限正数')
        self.takeoff_hold_time = float(
            self.get_parameter('takeoff_hold_time').value)
        if (
                not math.isfinite(self.takeoff_hold_time)
                or self.takeoff_hold_time <= 0.0):
            raise ValueError('takeoff_hold_time必须是有限正数')
        self.takeoff_position = (0.0, 0.0, -self.takeoff_height)

        transformer = CoordinateTransformer(
            origin_north=self.get_parameter('origin_north').value,
            origin_east=self.get_parameter('origin_east').value,
            origin_down=self.get_parameter('origin_down').value,
            yaw_offset=self.get_parameter('yaw_offset').value,
        )
        self.logic = MissionOffboardLogic(
            transformer=transformer,
            target_timeout=self.get_parameter('target_timeout').value,
            mission_status_timeout=self.get_parameter(
                'mission_status_timeout').value,
            local_position_timeout=self.get_parameter(
                'local_position_timeout').value,
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos,
        )
        self.trajectory_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos,
        )
        self.command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos,
        )
        self.target_subscription = self.create_subscription(
            MissionTrajectoryPoint,
            '/mission/trajectory_point',
            self.target_callback,
            10,
        )
        self.mission_status_subscription = self.create_subscription(
            MissionStatus,
            '/mission/status',
            self.mission_status_callback,
            10,
        )
        self.local_position_subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_callback,
            px4_qos,
        )
        self.vehicle_status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            px4_qos,
        )
        self.command_ack_subscription = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.command_ack_callback,
            px4_qos,
        )
        self.timer = self.create_timer(
            self.TIMER_PERIOD,
            self.timer_callback,
        )
        self.output_active = False
        self.control_state = self.WAIT
        self.prestream_started_at: Optional[float] = None
        self.takeoff_hold_started_at: Optional[float] = None
        self.horizontal_stable_started_at: Optional[float] = None
        self.local_velocity_x: Optional[float] = None
        self.local_velocity_y: Optional[float] = None
        self.local_position_z: Optional[float] = None
        self.last_command_time: Optional[float] = None
        self.px4_offboard = False
        self.px4_armed = False
        self.px4_auto_land = False
        self.land_state_active = False
        self.land_command_sent = False
        self.land_command_failed = False
        self.last_land_command_time: Optional[float] = None
        self.auto_land_logged = False
        self.ground_contact_started_at: Optional[float] = None
        self.disarm_command_sent = False
        self.get_logger().info(
            '任务Offboard控制节点已启动；控制状态：WAIT')

    def now_seconds(self) -> float:
        """返回ROS时钟秒数。"""
        return self.get_clock().now().nanoseconds * 1e-9

    def timestamp(self) -> int:
        """返回PX4消息使用的微秒时间戳。"""
        return self.get_clock().now().nanoseconds // 1000

    def target_callback(self, message: MissionTrajectoryPoint) -> None:
        """接收任务地图轨迹点并转换为PX4 NED目标。"""
        if message.header.frame_id not in ('', 'map'):
            self.get_logger().warning(
                '忽略非map坐标系任务目标：%s'
                % message.header.frame_id,
                throttle_duration_sec=5.0,
            )
            return
        try:
            self.logic.update_target(
                message.x,
                message.y,
                message.z,
                message.yaw,
                self.now_seconds(),
            )
        except ValueError as error:
            self.get_logger().error('忽略非法任务目标：%s' % error)

    def mission_status_callback(self, message: MissionStatus) -> None:
        """接收任务状态，IDLE和FAILSAFE会禁止目标输出。"""
        self.logic.update_mission_status(
            message.state,
            self.now_seconds(),
        )

    def local_position_callback(
            self, message: VehicleLocalPosition) -> None:
        """接收PX4本地位置并检查有效性。"""
        self.logic.update_local_position(
            message.x,
            message.y,
            message.z,
            message.xy_valid,
            message.z_valid,
            self.now_seconds(),
        )
        if all(math.isfinite(float(value)) for value in (
                message.vx, message.vy)):
            self.local_velocity_x = float(message.vx)
            self.local_velocity_y = float(message.vy)
        else:
            self.local_velocity_x = None
            self.local_velocity_y = None
        if message.z_valid and math.isfinite(float(message.z)):
            self.local_position_z = float(message.z)
        else:
            self.local_position_z = None

    def vehicle_status_callback(self, message: VehicleStatus) -> None:
        """保存PX4当前Offboard、解锁和自动降落状态。"""
        self.px4_offboard = (
            message.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.px4_armed = (
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.px4_auto_land = (
            message.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND)

    def command_ack_callback(self, message: VehicleCommandAck) -> None:
        """记录Offboard、解锁和降落命令的PX4确认结果。"""
        supported_commands = (
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        )
        if message.command not in supported_commands:
            return

        if message.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.land_command_failed = message.result not in (
                VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
                VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
            )

        if message.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
            self.get_logger().info(
                'PX4已接受VehicleCommand：%d' % message.command)
        else:
            self.get_logger().warning(
                'PX4拒绝或未完成VehicleCommand：command=%d result=%d'
                % (message.command, message.result))

    def publish_offboard_mode(self) -> None:
        """发布仅启用位置控制的PX4 Offboard心跳。"""
        message = OffboardControlMode()
        message.timestamp = self.timestamp()
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False
        self.offboard_publisher.publish(message)

    def publish_trajectory_setpoint(
            self,
            position: Optional[tuple[float, float, float]] = None,
            yaw: Optional[float] = None,
    ) -> None:
        """发布NED位置目标，速度、加速度和jerk全部设为NaN。"""
        if position is None:
            target = self.logic.target
            if target is None:
                return
            position = (target.north, target.east, target.down)
            yaw = self.logic.target_yaw
        elif yaw is None:
            yaw = 0.0

        message = TrajectorySetpoint()
        message.timestamp = self.timestamp()
        nan = math.nan
        message.position = list(position)
        message.velocity = [nan, nan, nan]
        message.acceleration = [nan, nan, nan]
        message.jerk = [nan, nan, nan]
        message.yaw = float(yaw)
        message.yawspeed = nan
        self.trajectory_publisher.publish(message)

    def publish_vehicle_command(
            self,
            command: int,
            param1: float = 0.0,
            param2: float = 0.0,
    ) -> None:
        """向PX4发送模式切换或解锁命令。"""
        message = VehicleCommand()
        message.timestamp = self.timestamp()
        message.param1 = float(param1)
        message.param2 = float(param2)
        message.command = int(command)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_publisher.publish(message)

    def set_control_state(self, state: str) -> None:
        """切换并记录内部PX4控制状态。"""
        if state == self.control_state:
            return
        self.control_state = state
        self.last_command_time = None
        self.get_logger().info('PX4控制状态：%s' % state)

    def command_due(self, now_seconds: float) -> bool:
        """判断VehicleCommand是否需要首次发送或定时重发。"""
        if self.last_command_time is None:
            return True
        return (
            now_seconds - self.last_command_time
            >= self.COMMAND_RETRY_INTERVAL
        )

    def step_control_state(
            self, now_seconds: float, target_allowed: bool) -> None:
        """推进预流、Offboard切换、解锁、起飞保持和任务执行状态。"""
        if self.control_state == self.WAIT:
            if target_allowed:
                self.prestream_started_at = now_seconds
                self.set_control_state(self.PRESTREAM)
            return

        if self.control_state == self.PRESTREAM:
            if not target_allowed:
                self.prestream_started_at = None
                self.set_control_state(self.WAIT)
                return
            if (
                    self.prestream_started_at is not None
                    and now_seconds - self.prestream_started_at
                    >= self.PRESTREAM_DURATION):
                self.set_control_state(self.REQUEST_OFFBOARD)
            return

        if self.control_state == self.REQUEST_OFFBOARD:
            if self.px4_offboard:
                self.set_control_state(self.REQUEST_ARM)
                return
            if self.command_due(now_seconds):
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    param1=1.0,
                    param2=6.0,
                )
                self.last_command_time = now_seconds
                self.get_logger().info('已请求PX4进入Offboard模式')
            return

        if self.control_state == self.REQUEST_ARM:
            if not self.px4_offboard:
                self.set_control_state(self.REQUEST_OFFBOARD)
                return
            if self.px4_armed:
                self.takeoff_hold_started_at = now_seconds
                self.horizontal_stable_started_at = None
                self.set_control_state(self.TAKEOFF_HOLD)
                self.get_logger().info(
                    'TAKEOFF_HOLD at height=%.2f m，保持时间=%.1f s'
                    % (self.takeoff_height, self.takeoff_hold_time))
                return
            if self.command_due(now_seconds):
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,
                )
                self.last_command_time = now_seconds
                self.get_logger().info('已请求PX4解锁')
            return

        if self.control_state == self.TAKEOFF_HOLD:
            if not self.px4_offboard:
                self.set_control_state(self.REQUEST_OFFBOARD)
                return
            if not self.px4_armed:
                self.set_control_state(self.REQUEST_ARM)
                return
            hold_complete = (
                self.takeoff_hold_started_at is not None
                and now_seconds - self.takeoff_hold_started_at
                >= self.takeoff_hold_time
            )
            velocity_stable = (
                self.local_velocity_x is not None
                and self.local_velocity_y is not None
                and abs(self.local_velocity_x)
                < self.HORIZONTAL_SPEED_THRESHOLD
                and abs(self.local_velocity_y)
                < self.HORIZONTAL_SPEED_THRESHOLD
            )
            if not hold_complete or not velocity_stable:
                self.horizontal_stable_started_at = None
                return
            if self.horizontal_stable_started_at is None:
                self.horizontal_stable_started_at = now_seconds
                self.get_logger().info(
                    '水平速度已低于0.1 m/s，开始1秒稳定计时')
                return
            if (
                    now_seconds - self.horizontal_stable_started_at
                    >= self.HORIZONTAL_STABILITY_DURATION):
                self.set_control_state(self.MISSION_EXECUTE)

    def handle_land_state(self, now_seconds: float) -> None:
        """在LAND首次发送降落命令，失败时最多每秒重试一次。"""
        if self.logic.mission_state != MissionStatus.LAND:
            self.land_state_active = False
            self.land_command_sent = False
            self.land_command_failed = False
            self.last_land_command_time = None
            self.auto_land_logged = False
            return

        if not self.land_state_active:
            self.land_state_active = True

        should_send = not self.land_command_sent
        should_retry = (
            self.land_command_failed
            and self.last_land_command_time is not None
            and now_seconds - self.last_land_command_time
            >= self.COMMAND_RETRY_INTERVAL
        )
        if should_send or should_retry:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.land_command_sent = True
            self.land_command_failed = False
            self.last_land_command_time = now_seconds
            self.get_logger().info('已发送PX4降落命令')

        if self.px4_auto_land and not self.auto_land_logged:
            self.get_logger().info('PX4进入自动降落')
            self.auto_land_logged = True

    def handle_ground_disarm(self, now_seconds: float) -> None:
        """自动降落近地持续2秒后发送一次锁桨命令。"""
        if self.logic.mission_state != MissionStatus.LAND:
            self.ground_contact_started_at = None
            self.disarm_command_sent = False
            return

        local_position_fresh = self.logic._fresh(
            now_seconds,
            self.logic.local_position_time,
            self.logic.local_position_timeout,
        )
        near_ground = (
            self.px4_auto_land
            and self.px4_armed
            and local_position_fresh
            and self.local_position_z is not None
            and self.local_position_z > self.GROUND_Z_THRESHOLD
        )
        if not near_ground:
            self.ground_contact_started_at = None
            return

        if self.ground_contact_started_at is None:
            self.ground_contact_started_at = now_seconds
            self.get_logger().info(
                '自动降落已接近地面，开始2秒锁桨确认')
            return

        if (
                not self.disarm_command_sent
                and now_seconds - self.ground_contact_started_at
                >= self.GROUND_DISARM_DURATION):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0,
            )
            self.disarm_command_sent = True
            self.get_logger().info('已发送PX4锁桨命令')

    def timer_callback(self) -> None:
        """以10 Hz发送Offboard心跳并推进自动起飞控制状态。"""
        now_seconds = self.now_seconds()
        allowed = self.logic.target_allowed(now_seconds)

        self.publish_offboard_mode()
        if self.control_state == self.TAKEOFF_HOLD:
            self.publish_trajectory_setpoint(
                position=self.takeoff_position,
                yaw=0.0,
            )
        elif (
                allowed
                or self.control_state in (
                    self.PRESTREAM,
                    self.REQUEST_OFFBOARD,
                    self.REQUEST_ARM,
                    self.MISSION_EXECUTE,
                )
        ):
            self.publish_trajectory_setpoint()
        self.step_control_state(now_seconds, allowed)
        self.handle_land_state(now_seconds)
        self.handle_ground_disarm(now_seconds)

        if allowed != self.output_active:
            if allowed:
                self.get_logger().info('任务目标输出已启用')
            else:
                self.get_logger().warning('任务目标输出已停止')
            self.output_active = allowed


def main(args=None) -> None:
    """启动任务PX4 Offboard位置控制节点。"""
    rclpy.init(args=args)
    node = MissionOffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
