"""在PX4 SITL中将视觉机体速度转换为NED Offboard速度设定值。."""

import math

from geometry_msgs.msg import TwistStamped

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLandDetected
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleStatus

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

from .qr_mission import QRMission


class VisionOffboardLogic:
    """与ROS接口解耦的坐标转换、高度控制和安全状态机。."""

    WAITING = 'WAITING'
    PRESTREAM = 'PRESTREAM'
    TAKEOFF = 'TAKEOFF'
    VISION_CONTROL = 'VISION_CONTROL'
    FAILSAFE = 'FAILSAFE'

    PRESTREAM_SECONDS = 1.0

    def __init__(self, simulation_mode=False, enable_offboard=False,
                 enable_auto_arm=False, target_altitude=2.0,
                 altitude_kp=0.8, max_vertical_velocity=0.5,
                 vision_timeout=0.3, max_horizontal_velocity=0.3,
                 takeoff_tolerance=0.15, takeoff_timeout=25.0,
                 px4_status_timeout=1.5, local_position_timeout=0.5):
        """保存控制参数并初始化安全状态。."""
        numeric = [target_altitude, altitude_kp, max_vertical_velocity,
                   vision_timeout, max_horizontal_velocity,
                   takeoff_tolerance, takeoff_timeout,
                   px4_status_timeout, local_position_timeout]
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError('控制参数必须是有限数值')
        if target_altitude <= 0.0 or altitude_kp <= 0.0:
            raise ValueError('target_altitude和altitude_kp必须大于0')
        if max_vertical_velocity <= 0.0:
            raise ValueError('max_vertical_velocity必须大于0')
        if vision_timeout <= 0.0 or max_horizontal_velocity <= 0.0:
            raise ValueError('视觉超时和水平限速必须大于0')
        if takeoff_tolerance <= 0.0:
            raise ValueError('takeoff_tolerance必须大于0')
        if takeoff_timeout <= 0.0:
            raise ValueError('takeoff_timeout必须大于0')
        if px4_status_timeout <= 0.0 or local_position_timeout <= 0.0:
            raise ValueError('PX4状态和本地位置超时必须大于0')

        self.simulation_mode = bool(simulation_mode)
        self.enable_offboard = bool(enable_offboard)
        self.enable_auto_arm = bool(enable_auto_arm)
        self.target_altitude = float(target_altitude)
        self.altitude_kp = float(altitude_kp)
        self.max_vertical_velocity = float(max_vertical_velocity)
        self.vision_timeout = float(vision_timeout)
        self.max_horizontal_velocity = float(max_horizontal_velocity)
        self.takeoff_tolerance = float(takeoff_tolerance)
        self.takeoff_timeout = float(takeoff_timeout)
        self.px4_status_timeout = float(px4_status_timeout)
        self.local_position_timeout = float(local_position_timeout)

        self.state = self.WAITING
        self.position_z = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0
        self.position_valid = False
        self.position_time = None
        self.status_time = None
        self.armed = False
        self.offboard_active = False
        self.px4_failsafe = False
        self.vision_forward = 0.0
        self.vision_left = 0.0
        self.vision_time = None
        self.vision_valid = False
        self.prestream_start = None
        self.takeoff_start = None
        self.target_z = None
        self.failsafe_reason = None
        self.mode_requested = False
        self.arm_requested = False
        self.pre_flight_checks_pass = False
        self.sitl_verified = False
        self.prestream_cycles = 0

    @staticmethod
    def flu_to_ned(body_forward, body_left, heading):
        """把ROS FLU机体水平速度旋转到PX4本地NED。."""
        body_right = -float(body_left)
        forward = float(body_forward)
        yaw = float(heading)
        north = math.cos(yaw) * forward - math.sin(yaw) * body_right
        east = math.sin(yaw) * forward + math.cos(yaw) * body_right
        return north, east

    @staticmethod
    def limit_horizontal(north, east, limit):
        """按二维速度模长限制水平速度。."""
        magnitude = math.hypot(north, east)
        if magnitude <= limit or magnitude == 0.0:
            return north, east
        scale = limit / magnitude
        return north * scale, east * scale

    @staticmethod
    def clamp(value, limit):
        """将标量限制到对称区间。."""
        return max(-limit, min(limit, value))

    def update_position(self, z, heading, valid, now_seconds, x=0.0, y=0.0):
        """更新PX4位置和航向。."""
        finite = math.isfinite(float(z)) and math.isfinite(float(heading))
        self.position_valid = bool(valid) and finite
        self.position_time = float(now_seconds)
        if self.position_valid:
            self.position_z = float(z)
            self.position_x = float(x)
            self.position_y = float(y)
            self.heading = float(heading)

    def position_acceptance(self, xy_valid, z_valid,
                            heading_good_for_control, heading):
        """按真实硬件或SITL规则判断位置与航向是否可用。."""
        heading_finite = math.isfinite(float(heading))
        base_valid = bool(xy_valid) and bool(z_valid) and heading_finite
        if not base_valid:
            return False, False
        if heading_good_for_control:
            return True, False
        if self.simulation_mode:
            return True, True
        return False, False

    def update_status(self, armed, offboard_active, failsafe, now_seconds,
                      pre_flight_checks_pass=True):
        """更新PX4解锁、导航和failsafe状态。."""
        self.armed = bool(armed)
        self.offboard_active = bool(offboard_active)
        self.px4_failsafe = bool(failsafe)
        self.pre_flight_checks_pass = bool(pre_flight_checks_pass)
        self.status_time = float(now_seconds)

    def update_sitl_evidence(self, verified):
        """Latch independently observed simulator clock evidence."""
        self.sitl_verified = self.sitl_verified or bool(verified)

    def update_vision(self, forward, left, valid, now_seconds):
        """更新FLU机体水平视觉速度。."""
        finite = math.isfinite(float(forward)) and math.isfinite(float(left))
        self.vision_valid = bool(valid) and finite
        self.vision_time = float(now_seconds)
        if self.vision_valid:
            self.vision_forward = float(forward)
            self.vision_left = float(left)
        else:
            self.vision_forward = 0.0
            self.vision_left = 0.0

    def auto_arm_allowed(self):
        """仅允许显式启用的仿真自动解锁。."""
        return all((self.simulation_mode, self.enable_offboard,
                    self.enable_auto_arm, self.sitl_verified,
                    self.pre_flight_checks_pass, not self.px4_failsafe,
                    self.position_valid, self.prestream_cycles >= 20))

    def px4_data_ready(self, now_seconds):
        """检查位置、航向和状态消息是否有效且新鲜。."""
        if not self.position_valid:
            return False
        if self.position_time is None or self.status_time is None:
            return False
        return all((
            now_seconds - self.position_time <= self.local_position_timeout,
            now_seconds - self.status_time <= self.px4_status_timeout,
        ))

    @staticmethod
    def timeout_reason(name, now_seconds, message_time, limit):
        """格式化基于本地ROS接收时间的超时原因。."""
        age = math.inf if message_time is None else now_seconds - message_time
        return '%s超时：age=%.2fs limit=%.2fs' % (name, age, limit)

    def px4_failure_reason(self, now_seconds):
        """返回PX4数据导致FAILSAFE的明确原因。."""
        if self.px4_failsafe:
            return 'PX4报告failsafe'
        if not self.position_valid:
            return '位置无效'
        if self.position_time is None:
            return self.timeout_reason(
                '本地位置', now_seconds, self.position_time,
                self.local_position_timeout)
        if now_seconds - self.position_time > self.local_position_timeout:
            return self.timeout_reason(
                '本地位置', now_seconds, self.position_time,
                self.local_position_timeout)
        if self.status_time is None:
            return self.timeout_reason(
                'PX4状态', now_seconds, self.status_time,
                self.px4_status_timeout)
        if now_seconds - self.status_time > self.px4_status_timeout:
            return self.timeout_reason(
                'PX4状态', now_seconds, self.status_time,
                self.px4_status_timeout)
        return None

    def altitude_error(self):
        """返回NED目标高度误差，向下为正。."""
        if self.target_z is None:
            return 0.0
        return self.target_z - self.position_z

    def height_velocity(self):
        """计算NED向下为正的垂直闭环速度。."""
        if self.target_z is None or not self.position_valid:
            return 0.0
        return self.clamp(
            self.altitude_kp * self.altitude_error(),
            self.max_vertical_velocity)

    def vision_ned_velocity(self, now_seconds):
        """视觉无效或超时时立即返回零水平速度。."""
        if not self.vision_valid or self.vision_time is None:
            return 0.0, 0.0
        if now_seconds - self.vision_time > self.vision_timeout:
            return 0.0, 0.0
        north, east = self.flu_to_ned(
            self.vision_forward, self.vision_left, self.heading)
        return self.limit_horizontal(
            north, east, self.max_horizontal_velocity)

    def step(self, now_seconds):
        """推进状态机并返回速度、模式请求和解锁请求。."""
        now = float(now_seconds)
        request_mode = False
        request_arm = False

        if self.state not in (self.WAITING, self.FAILSAFE):
            failure_reason = self.px4_failure_reason(now)
            if failure_reason is not None:
                self.state = self.FAILSAFE
                self.failsafe_reason = failure_reason

        if self.state == self.WAITING:
            if all((self.simulation_mode, self.enable_offboard,
                    self.px4_data_ready(now))):
                self.state = self.PRESTREAM
                self.prestream_start = now
                self.target_z = -abs(self.target_altitude)

        elif self.state == self.PRESTREAM:
            self.prestream_cycles += 1
            elapsed = now - self.prestream_start
            self.prestream_cycles = max(
                self.prestream_cycles, int(elapsed * 20.0))
            if elapsed >= self.PRESTREAM_SECONDS:
                if not self.mode_requested:
                    request_mode = True
                    self.mode_requested = True
                if self.auto_arm_allowed() and not self.arm_requested:
                    request_arm = True
                    self.arm_requested = True
                if self.offboard_active and self.armed:
                    self.state = self.TAKEOFF
                    self.takeoff_start = now

        elif self.state == self.TAKEOFF:
            if abs(self.altitude_error()) <= self.takeoff_tolerance:
                self.state = self.VISION_CONTROL
            elif now - self.takeoff_start > self.takeoff_timeout:
                self.state = self.FAILSAFE
                self.failsafe_reason = '起飞超时'

        if self.state == self.VISION_CONTROL:
            north, east = self.vision_ned_velocity(now)
            return (north, east, self.height_velocity(),
                    request_mode, request_arm)
        if self.state == self.TAKEOFF:
            return 0.0, 0.0, self.height_velocity(), request_mode, request_arm
        return 0.0, 0.0, 0.0, request_mode, request_arm


class VisionOffboardController(Node):
    """以20 Hz发布PX4速度模式心跳和安全速度设定值。."""

    def __init__(self):
        """创建参数、PX4接口和20 Hz控制定时器。."""
        super().__init__('vision_offboard_controller')
        self.declare_parameter('simulation_mode', False)
        self.declare_parameter('enable_offboard', False)
        self.declare_parameter('enable_auto_arm', False)
        self.declare_parameter('target_altitude', 2.0)
        self.declare_parameter('altitude_kp', 0.8)
        self.declare_parameter('max_vertical_velocity', 0.5)
        self.declare_parameter('vision_timeout', 0.3)
        self.declare_parameter('max_horizontal_velocity', 0.3)
        self.declare_parameter('takeoff_tolerance', 0.15)
        self.declare_parameter('takeoff_timeout', 25.0)
        self.declare_parameter('px4_status_timeout', 1.5)
        self.declare_parameter('local_position_timeout', 0.5)
        self.declare_parameter('task_mode', 'legacy')
        self.declare_parameter('target_qr_id', 7)
        self.declare_parameter('mission_event_timeout', 1.0)
        self.declare_parameter('mission_state_timeout', 30.0)
        self.declare_parameter('mission_timeout', 180.0)
        self.declare_parameter('qr_approach_area', 6000.0)
        self.declare_parameter('mission_align_error', 12.0)
        self.declare_parameter('transit_seconds', 4.0)
        self.declare_parameter('transit_speed', -0.20)
        self.declare_parameter('search_yaw_rate', 0.2)
        self.declare_parameter('scan_hold_seconds', 0.6)
        self.declare_parameter('scan_timeout', 8.0)
        self.declare_parameter('scan_max_retries', 2)
        self.declare_parameter('scan_position_kp', 0.8)
        self.declare_parameter('qr_mission_yaw', 0.0)
        self.declare_parameter('image_center_x', 160.0)
        self.declare_parameter('image_center_y', 120.0)
        self.declare_parameter(
            'vision_velocity_topic', '/control/vision_velocity')
        self.declare_parameter(
            'offboard_control_mode_topic',
            '/fmu/in/offboard_control_mode')
        self.declare_parameter(
            'trajectory_setpoint_topic',
            '/fmu/in/trajectory_setpoint')
        self.declare_parameter(
            'vehicle_command_topic', '/fmu/in/vehicle_command')
        self.declare_parameter(
            'vehicle_local_position_topic',
            '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter(
            'vehicle_status_topic', '/fmu/out/vehicle_status_v1')
        self.declare_parameter(
            'vehicle_land_detected_topic',
            '/fmu/out/vehicle_land_detected')
        self.declare_parameter(
            'qr_inventory_topic', '/vision/qr/inventory')
        self.declare_parameter(
            'qr_laser_topic', '/vision/qr/laser_aligned')
        self.declare_parameter(
            'selected_camera_topic', '/vision/selected_camera')
        self.declare_parameter(
            'selected_detection_topic', '/vision/selected_detection')

        names = ('simulation_mode', 'enable_offboard', 'enable_auto_arm',
                 'target_altitude', 'altitude_kp', 'max_vertical_velocity',
                 'vision_timeout', 'max_horizontal_velocity',
                 'takeoff_tolerance', 'takeoff_timeout',
                 'px4_status_timeout', 'local_position_timeout')
        values = {name: self.get_parameter(name).value for name in names}
        self.logic = VisionOffboardLogic(**values)
        self.task_mode = self.get_parameter('task_mode').value
        self.image_center_x = float(
            self.get_parameter('image_center_x').value)
        self.image_center_y = float(
            self.get_parameter('image_center_y').value)
        if self.task_mode not in ('legacy', 'qr_shelf'):
            raise ValueError('task_mode must be legacy or qr_shelf')
        self.mission = None
        if self.task_mode == 'qr_shelf':
            self.mission = QRMission(
                target_qr_id=self.get_parameter('target_qr_id').value,
                event_timeout=self.get_parameter(
                    'mission_event_timeout').value,
                state_timeout=self.get_parameter(
                    'mission_state_timeout').value,
                mission_timeout=self.get_parameter('mission_timeout').value,
                approach_area=self.get_parameter('qr_approach_area').value,
                align_error=self.get_parameter(
                    'mission_align_error').value,
                transit_seconds=self.get_parameter('transit_seconds').value,
                transit_speed=self.get_parameter('transit_speed').value,
                search_yaw_rate=self.get_parameter(
                    'search_yaw_rate').value,
                scan_hold_seconds=self.get_parameter(
                    'scan_hold_seconds').value,
                scan_timeout=self.get_parameter('scan_timeout').value,
                scan_max_retries=self.get_parameter(
                    'scan_max_retries').value)

        if self.logic.enable_auto_arm and not self.logic.simulation_mode:
            self.get_logger().error(
                '拒绝自动解锁：enable_auto_arm=true但simulation_mode=false')
        if self.logic.enable_offboard and not self.logic.simulation_mode:
            self.get_logger().error(
                '拒绝激活控制：enable_offboard只允许在simulation_mode中使用')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_publisher = self.create_publisher(
            OffboardControlMode,
            self.get_parameter('offboard_control_mode_topic').value,
            px4_qos)
        self.trajectory_publisher = self.create_publisher(
            TrajectorySetpoint,
            self.get_parameter('trajectory_setpoint_topic').value,
            px4_qos)
        self.command_publisher = self.create_publisher(
            VehicleCommand,
            self.get_parameter('vehicle_command_topic').value, px4_qos)
        self.vision_subscription = self.create_subscription(
            TwistStamped,
            self.get_parameter('vision_velocity_topic').value,
            self.vision_callback, 10)
        self.position_subscription = self.create_subscription(
            VehicleLocalPosition,
            self.get_parameter('vehicle_local_position_topic').value,
            self.position_callback, px4_qos)
        self.status_subscription = self.create_subscription(
            VehicleStatus,
            self.get_parameter('vehicle_status_topic').value,
            self.status_callback, px4_qos)
        self.land_subscription = self.create_subscription(
            VehicleLandDetected,
            self.get_parameter('vehicle_land_detected_topic').value,
            self.land_callback, px4_qos)
        self.clock_subscription = self.create_subscription(
            Clock, '/clock', self.clock_callback, 10)
        self.inventory_subscription = self.create_subscription(
            String, self.get_parameter('qr_inventory_topic').value,
            self.inventory_callback, 10)
        self.laser_subscription = self.create_subscription(
            Bool, self.get_parameter('qr_laser_topic').value,
            self.laser_callback, 10)
        self.camera_subscription = self.create_subscription(
            String, self.get_parameter('selected_camera_topic').value,
            self.camera_callback, 10)
        self.selected_detection_subscription = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('selected_detection_topic').value,
            self.selected_detection_callback, 10)
        self.mission_state_publisher = self.create_publisher(
            String, '/control/qr_mission_state', 10)
        # PX4 的 Offboard 丢失保护要求心跳持续到达。双路图像渲染和检测
        # 会产生短时调度抖动，20 Hz 为默认超时保留充足余量。
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.last_logged_state = None
        self.land_requested = False
        self.disarm_requested = False
        self.last_landed = False
        self.last_mission_state = None
        self.sitl_heading_warning_emitted = False
        self.get_logger().info(
            '视觉Offboard控制器已安全启动：simulation=%s offboard=%s auto_arm=%s'
            % (self.logic.simulation_mode, self.logic.enable_offboard,
               self.logic.enable_auto_arm))

    def now_seconds(self):
        """返回ROS时钟秒数。."""
        return self.get_clock().now().nanoseconds * 1e-9

    def timestamp(self):
        """返回PX4消息使用的微秒时间戳。."""
        return self.get_clock().now().nanoseconds // 1000

    def vision_callback(self, message):
        """接收base_link中的ROS FLU视觉速度。."""
        valid_frame = message.header.frame_id in ('', 'base_link')
        if not valid_frame:
            self.get_logger().warning(
                '忽略非base_link视觉速度并立即清零',
                throttle_duration_sec=5.0)
        self.logic.update_vision(
            message.twist.linear.x,
            message.twist.linear.y,
            valid_frame,
            self.now_seconds(),
        )

    def position_callback(self, message):
        """接收并验证PX4本地位置与heading。."""
        valid, used_sitl_heading = self.logic.position_acceptance(
            message.xy_valid,
            message.z_valid,
            message.heading_good_for_control,
            message.heading,
        )
        if used_sitl_heading and not self.sitl_heading_warning_emitted:
            self.get_logger().warning(
                'SITL模式：使用有限heading，忽略heading_good_for_control=false')
            self.sitl_heading_warning_emitted = True
        self.logic.update_position(
            message.z, message.heading, valid, self.now_seconds(),
            message.x, message.y)
        if self.mission is not None and valid:
            self.mission.update_position(message.x, message.y, message.z)

    def status_callback(self, message):
        """接收PX4解锁、Offboard和failsafe状态。."""
        self.logic.update_status(
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            message.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            message.failsafe,
            self.now_seconds(),
            message.pre_flight_checks_pass,
        )
        if getattr(self, 'mission', None) is not None:
            self.mission.update_flight(
                self.logic.armed, self.last_landed, self.now_seconds())

    def clock_callback(self, message):
        """Use a progressing external simulation clock as SITL evidence."""
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        if math.isfinite(stamp) and stamp > 0.0:
            self.logic.update_sitl_evidence(True)

    def land_callback(self, message):
        """Consume the real PX4 landing detector."""
        self.last_landed = bool(message.landed)
        if self.mission is not None:
            self.mission.update_flight(
                self.logic.armed, self.last_landed, self.now_seconds())

    def inventory_callback(self, message):
        """Drive inventory completion and target acquisition from JSON."""
        if self.mission is None:
            return
        import json
        try:
            data = json.loads(message.data)
            records = data.get('records', [])
            image_size = data.get('image_size', [320, 240])
            target = next((
                item for item in records
                if int(item.get('qr_id', -1)) ==
                self.mission.target_qr_id), None)
            target_visible = self.mission.target_qr_id in [
                int(value) for value in data.get('visible_ids', [])]
            complete = bool(data.get('complete', False))
            if target is not None:
                center = target.get('image_center', [math.inf, math.inf])
                error_x = (
                    float(center[0]) - float(image_size[0]) / 2.0)
                error_y = (
                    float(center[1]) - float(image_size[1]) / 2.0)
                error = math.hypot(
                    error_x, error_y)
                area = float(target.get('area', 0.0))
            else:
                area, error = 0.0, math.inf
                error_x, error_y = math.inf, math.inf
            self.mission.update_inventory(
                complete, target is not None, area, error,
                self.now_seconds(), data.get('count', 0), target_visible,
                error_x, error_y)
        except (KeyError, TypeError, ValueError):
            self.get_logger().warning('ignored malformed QR inventory')

    def laser_callback(self, message):
        if self.mission is not None:
            self.mission.update_laser(message.data, self.now_seconds())

    def camera_callback(self, message):
        if self.mission is not None:
            self.mission.update_camera(message.data, self.now_seconds())

    def selected_detection_callback(self, message):
        """Track real down-camera alignment error."""
        if self.mission is None or self.mission.selected_camera != 'down':
            return
        values = list(message.data)
        valid = len(values) == 7 and values[0] == 1.0 and all(
            math.isfinite(float(value)) for value in values)
        error = math.hypot(
            values[1] - self.image_center_x,
            values[2] - self.image_center_y) \
            if valid else math.inf
        self.mission.update_down(valid, error, self.now_seconds())

    def publish_offboard_mode(self):
        """发布仅启用速度控制的Offboard心跳。."""
        message = OffboardControlMode()
        message.timestamp = self.timestamp()
        message.position = False
        message.velocity = True
        message.acceleration = False
        message.attitude = False
        message.body_rate = False
        message.thrust_and_torque = False
        message.direct_actuator = False
        self.offboard_publisher.publish(message)

    def publish_setpoint(self, north, east, down, yaw_rate=math.nan,
                         yaw=math.nan):
        """发布全部位置无效、仅速度有效的NED设定值。."""
        message = TrajectorySetpoint()
        message.timestamp = self.timestamp()
        nan = float('nan')
        message.position = [nan, nan, nan]
        message.velocity = [north, east, down]
        message.acceleration = [nan, nan, nan]
        message.jerk = [nan, nan, nan]
        message.yaw = yaw
        message.yawspeed = yaw_rate
        self.trajectory_publisher.publish(message)

    def publish_command(self, command, param1=0.0, param2=0.0):
        """发布PX4模式或仿真解锁命令。."""
        message = VehicleCommand()
        message.timestamp = self.timestamp()
        message.param1 = param1
        message.param2 = param2
        message.command = command
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_publisher.publish(message)

    def timer_callback(self):
        """以20 Hz推进状态机并发布心跳与速度。."""
        if not self.logic.enable_offboard:
            if self.last_logged_state is None:
                self.get_logger().info(
                    'enable_offboard=false：仅监视输入，不发布PX4控制消息')
                self.last_logged_state = self.logic.state
            return

        north, east, down, request_mode, request_arm = self.logic.step(
            self.now_seconds())
        mission_output = None
        if getattr(self, 'mission', None) is not None:
            if self.logic.state == self.logic.PRESTREAM:
                self.mission.start(self.now_seconds())
            mission_output = self.mission.step(
                self.now_seconds(), self.logic.state)
            state_message = String()
            import json
            state_message.data = json.dumps({
                'state': self.mission.state,
                'scan_index': self.mission.scan_index,
                'expected_qr_id': self.mission.scan_order[
                    min(self.mission.scan_index, 23)],
                'hold': self.mission.state == self.mission.QR_SCAN_CONFIRM,
                'retry_count': self.mission.retry_count,
                'scan_position': list(self.mission.scan_point())})
            self.mission_state_publisher.publish(state_message)
            if self.mission.state != self.last_mission_state:
                self.get_logger().info(
                    'QR任务状态：%s，原因：%s' % (
                        self.mission.state, self.mission.reason))
                self.last_mission_state = self.mission.state
            if self.mission.state == self.mission.FAILSAFE and \
                    self.logic.state != self.logic.FAILSAFE:
                self.logic.state = self.logic.FAILSAFE
                self.logic.failsafe_reason = self.mission.reason
        if self.logic.state == self.logic.FAILSAFE:
            if not self.land_requested and self.logic.armed:
                self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.land_requested = True
                self.get_logger().error(
                    'FAILSAFE：已请求PX4正常降落并停止Offboard输出')
            if self.logic.state != self.last_logged_state:
                self.get_logger().error(
                    '控制状态：FAILSAFE，原因：%s'
                    % self.logic.failsafe_reason)
                self.last_logged_state = self.logic.state
            return

        yaw_rate = math.nan
        yaw = math.nan
        if mission_output is not None:
            if mission_output.request_land:
                if not self.land_requested:
                    self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.land_requested = True
                    self.get_logger().info(
                        'QR mission: requested PX4 Land once')
                return
            if mission_output.request_disarm:
                if not self.disarm_requested:
                    self.publish_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        param1=0.0)
                    self.disarm_requested = True
                return
            yaw_rate = mission_output.yaw_rate
            # Shelf bearing is a deployment parameter, not Gazebo truth.
            # Holding it also prevents an unconstrained yaw drift during
            # takeoff/search when PX4's magnetic heading settles.
            yaw = float(self.get_parameter('qr_mission_yaw').value)
            if mission_output.use_vision:
                north, east = self.logic.vision_ned_velocity(
                    self.now_seconds())
                if mission_output.scan_position:
                    target_d = mission_output.scan_position[2]
                    gain = float(
                        self.get_parameter('scan_position_kp').value)
                    down = self.logic.clamp(
                        gain * (target_d - self.logic.position_z),
                        self.logic.max_vertical_velocity)
            elif mission_output.scan_position:
                target_n, target_e, target_d = mission_output.scan_position
                gain = float(self.get_parameter('scan_position_kp').value)
                north = gain * (target_n - self.logic.position_x)
                east = gain * (target_e - self.logic.position_y)
                down = self.logic.clamp(
                    gain * (target_d - self.logic.position_z),
                    self.logic.max_vertical_velocity)
                north, east = self.logic.limit_horizontal(
                    north, east, self.logic.max_horizontal_velocity)
            else:
                north, east = self.logic.flu_to_ned(
                    mission_output.forward, mission_output.left,
                    self.logic.heading)
                north, east = self.logic.limit_horizontal(
                    north, east, self.logic.max_horizontal_velocity)
            if math.isfinite(mission_output.vertical):
                down = mission_output.vertical

        self.publish_offboard_mode()
        self.publish_setpoint(north, east, down, yaw_rate, yaw)

        if request_mode:
            self.publish_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
            self.get_logger().info('已请求SITL进入Offboard模式')
        if request_arm and self.logic.auto_arm_allowed():
            self.publish_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self.get_logger().warning('已发送仅限SITL的自动解锁命令')

        if self.logic.state != self.last_logged_state:
            self.get_logger().info('控制状态：%s' % self.logic.state)
            self.last_logged_state = self.logic.state

        if self.logic.state == self.logic.TAKEOFF:
            self.get_logger().info(
                'TAKEOFF: current_z=%.3f target_z=%.3f '
                'altitude_error=%.3f vertical_velocity=%.3f'
                % (self.logic.position_z, self.logic.target_z,
                   self.logic.altitude_error(), self.logic.height_velocity()),
                throttle_duration_sec=1.0,
            )


def main(args=None):
    """启动视觉速度到PX4仿真的控制节点。."""
    rclpy.init(args=args)
    node = VisionOffboardController()
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
