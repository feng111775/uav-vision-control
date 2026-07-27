"""H题任务规划层的ROS 2接口节点。"""

import math

from geometry_msgs.msg import PoseStamped
from mission_interfaces.msg import MissionStatus
from mission_interfaces.msg import TrajectoryPoint as TrajectoryPointMessage
from px4_msgs.msg import VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_srvs.srv import Trigger

from .coverage_planner import CoveragePlanner
from .map_manager import GridMap
from .mission_manager import MissionManager, MissionState
from .trajectory_generator import TrajectoryGenerator, TrajectoryPoint


class MissionManagerNode(Node):
    """封装纯Python任务规划流程并提供ROS 2任务接口。"""

    TIMER_PERIOD = 0.1
    POSITION_CONTROL_MODE = 0

    STATE_VALUES = {
        MissionState.IDLE: MissionStatus.IDLE,
        MissionState.TAKEOFF: MissionStatus.TAKEOFF,
        MissionState.EXECUTE: MissionStatus.EXECUTE,
        MissionState.RETURN: MissionStatus.RETURN,
        MissionState.LAND: MissionStatus.LAND,
        MissionState.COMPLETE: MissionStatus.COMPLETE,
        MissionState.FAILSAFE: MissionStatus.FAILSAFE,
    }
    STATE_LOGS = {
        MissionState.TAKEOFF: '开始自动起飞',
        MissionState.EXECUTE: '开始执行覆盖巡查',
        MissionState.RETURN: '覆盖完成，返回起飞点',
        MissionState.LAND: '开始自动降落',
        MissionState.COMPLETE: '任务完成',
    }

    def __init__(self):
        """创建任务、ROS 2发布订阅接口和任务控制服务。"""
        super().__init__('mission_manager_node')

        self.declare_parameter('takeoff_x', 20.0)
        self.declare_parameter('takeoff_y', -15.0)
        self.declare_parameter('takeoff_height', 2.0)
        self.declare_parameter('patrol_height', 2.0)
        self.declare_parameter('patrol_speed', 5.0)
        self.declare_parameter(
            'no_fly_cells', ['A3B2', 'A3B3', 'A3B4'])
        self.declare_parameter('demo_mode', False)
        self.declare_parameter('demo_waypoints', 20)
        self.takeoff_x = float(self.get_parameter('takeoff_x').value)
        self.takeoff_y = float(self.get_parameter('takeoff_y').value)
        if not all(math.isfinite(value) for value in (
                self.takeoff_x, self.takeoff_y)):
            raise ValueError('起飞点坐标必须是有限数值')
        self.takeoff_height = self._positive_height_parameter(
            'takeoff_height')
        self.patrol_height = self._positive_height_parameter(
            'patrol_height')
        self.patrol_speed = self._positive_height_parameter(
            'patrol_speed')
        self.no_fly_cells = list(
            self.get_parameter('no_fly_cells').value)
        self.demo_mode = bool(self.get_parameter('demo_mode').value)
        self.demo_waypoints = int(
            self.get_parameter('demo_waypoints').value)
        if self.demo_waypoints <= 0:
            raise ValueError('demo_waypoints必须是正整数')

        self.grid_map = None
        self.plan_result = None
        self.trajectory = []
        self.loaded_coverage_waypoints = 0
        self.manager = None
        self.last_logged_state = None
        self._initialize_mission()

        self.target_publisher = self.create_publisher(
            TrajectoryPointMessage,
            '/mission/trajectory_point',
            10,
        )
        self.status_publisher = self.create_publisher(
            MissionStatus,
            '/mission/status',
            10,
        )
        self.position_subscription = self.create_subscription(
            PoseStamped,
            '/mission/current_position',
            self.position_callback,
            10,
        )
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.vehicle_status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            px4_qos,
        )
        self.start_service = self.create_service(
            Trigger,
            '/mission/start',
            self.start_callback,
        )
        self.reset_service = self.create_service(
            Trigger,
            '/mission/reset',
            self.reset_callback,
        )
        self.abort_service = self.create_service(
            Trigger,
            '/mission/abort',
            self.abort_callback,
        )
        self.timer = self.create_timer(
            self.TIMER_PERIOD, self.timer_callback)

        self.get_logger().info(
            '任务管理节点已启动：覆盖航点=%d，轨迹点=%d，'
            'takeoff=(%.2f, %.2f, %.2f)，patrol_height=%.2f，'
            'patrol_speed=%.2f，demo_mode=%s'
            % (
                self.loaded_coverage_waypoints,
                len(self.trajectory),
                self.takeoff_x,
                self.takeoff_y,
                self.takeoff_height,
                self.patrol_height,
                self.patrol_speed,
                self.demo_mode,
            )
        )

    def _positive_height_parameter(self, name):
        """读取并验证有限正数高度参数。"""
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError('%s必须是有限正数' % name)
        return value

    def _initialize_mission(self):
        """重新创建地图、规划器、轨迹生成器和任务管理器。"""
        grid_map = GridMap(
            rows=7,
            cols=9,
            cell_size=5.0,
            no_fly_cells=self.no_fly_cells,
        )
        planner = CoveragePlanner(
            patrol_height=self.patrol_height,
            speed=self.patrol_speed,
        )
        plan_result = planner.plan(grid_map)
        if not plan_result.success:
            raise RuntimeError(
                '覆盖规划失败：%s' % plan_result.failure_reason)

        generator = TrajectoryGenerator(speed=self.patrol_speed)
        coverage_trajectory = generator.generate(plan_result.waypoints)
        if self.demo_mode:
            coverage_trajectory = coverage_trajectory[:self.demo_waypoints]
        takeoff_point = TrajectoryPoint(
            timestamp=0.0,
            x=self.takeoff_x,
            y=self.takeoff_y,
            z=self.takeoff_height,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw=0.0,
            waypoint_id=len(plan_result.waypoints),
        )
        trajectory = [takeoff_point] + coverage_trajectory
        manager = MissionManager()
        manager.load_trajectory(trajectory)
        manager.return_target = TrajectoryPoint(
            timestamp=0.0,
            x=self.takeoff_x,
            y=self.takeoff_y,
            z=self.takeoff_height,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw=0.0,
            waypoint_id=len(trajectory),
        )

        self.grid_map = grid_map
        self.plan_result = plan_result
        self.trajectory = trajectory
        self.loaded_coverage_waypoints = len(coverage_trajectory)
        self.manager = manager
        self.last_logged_state = None

    def position_callback(self, message):
        """接收地图坐标系位置，并推进一次任务状态。"""
        position = message.pose.position
        self.manager.update_position(position.x, position.y, position.z)
        self.manager.step()

    def vehicle_status_callback(self, message):
        """接收PX4自动降落和锁桨状态并推进任务。"""
        self.manager.update_landing_status(
            auto_land=(
                message.nav_state
                == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            ),
            disarmed=(
                message.arming_state
                == VehicleStatus.ARMING_STATE_DISARMED
            ),
        )
        self.manager.step()

    def start_callback(self, request, response):
        """启动已加载任务。"""
        del request
        try:
            self.manager.start()
        except (RuntimeError, ValueError) as error:
            response.success = False
            response.message = str(error)
        else:
            response.success = True
            response.message = '任务已启动'
        return response

    def reset_callback(self, request, response):
        """重新创建并加载默认任务。"""
        del request
        try:
            self._initialize_mission()
        except (RuntimeError, ValueError) as error:
            response.success = False
            response.message = str(error)
        else:
            response.success = True
            response.message = '任务已重置'
        return response

    def abort_callback(self, request, response):
        """中止当前任务并进入FAILSAFE。"""
        del request
        self.manager.enter_failsafe('收到/mission/abort请求')
        response.success = True
        response.message = '任务已中止并进入FAILSAFE'
        return response

    @staticmethod
    def _duration_message(timestamp):
        """将浮点秒转换为ROS Duration消息字段。"""
        seconds = max(0.0, float(timestamp))
        whole_seconds = math.floor(seconds)
        nanoseconds = round((seconds - whole_seconds) * 1_000_000_000)
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        return int(whole_seconds), int(nanoseconds)

    def publish_current_target(self):
        """发布当前任务目标；无目标状态不发布轨迹点。"""
        target = self.manager.get_current_target()
        if target is None:
            return

        message = TrajectoryPointMessage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        seconds, nanoseconds = self._duration_message(target.timestamp)
        message.time_from_start.sec = seconds
        message.time_from_start.nanosec = nanoseconds
        message.waypoint_id = target.waypoint_id
        message.x = target.x
        message.y = target.y
        message.z = target.z
        message.vx = target.vx
        message.vy = target.vy
        message.vz = target.vz
        message.yaw = target.yaw
        message.control_mode = self.POSITION_CONTROL_MODE
        self.target_publisher.publish(message)

    def _current_cell(self, target):
        """根据轨迹点保存的覆盖航点编号返回方格代码。"""
        if target is None:
            return ''
        waypoint_id = target.waypoint_id
        if not 0 <= waypoint_id < len(self.plan_result.waypoints):
            return ''
        return self.plan_result.waypoints[waypoint_id]['cell_code']

    def publish_status(self):
        """发布当前任务状态、目标和进度。"""
        target = self.manager.get_current_target()
        total = len(self.trajectory)
        index = min(self.manager.current_waypoint_index, total)

        message = MissionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.state = self.STATE_VALUES[self.manager.current_state]
        message.mission_loaded = bool(self.manager.trajectory)
        message.target_available = target is not None
        message.current_cell = self._current_cell(target)
        message.current_waypoint_index = index
        message.total_waypoints = total
        message.progress = float(index / total) if total else 0.0
        distance = self.manager.distance_to_target()
        message.distance_to_target = (
            distance if math.isfinite(distance) else -1.0)
        message.failure_reason = self.manager.failsafe_reason
        self.status_publisher.publish(message)

    def timer_callback(self):
        """以10 Hz推进状态并发布目标轨迹点和任务状态。"""
        self.manager.step()
        self.publish_current_target()
        self.publish_status()

        if self.manager.current_state != self.last_logged_state:
            message = self.STATE_LOGS.get(
                self.manager.current_state,
                '任务状态：%s' % self.manager.current_state.value,
            )
            self.get_logger().info(message)
            self.last_logged_state = self.manager.current_state


def main(args=None):
    """启动ROS 2任务管理节点。"""
    rclpy.init(args=args)
    node = MissionManagerNode()
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
