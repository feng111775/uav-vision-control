"""2025电赛H题任务流程的纯Python状态管理逻辑。"""

from enum import Enum
import math

try:
    from .trajectory_generator import TrajectoryPoint
except ImportError:
    from trajectory_generator import TrajectoryPoint


class MissionState(Enum):
    """任务管理器支持的状态。"""

    IDLE = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    EXECUTE = 'EXECUTE'
    RETURN = 'RETURN'
    LAND = 'LAND'
    COMPLETE = 'COMPLETE'
    FAILSAFE = 'FAILSAFE'


class MissionManager:
    """管理起飞、轨迹执行、返回和降落的通用任务流程。"""

    def __init__(self, arrival_radius=1.0):
        """创建空闲任务管理器，并设置默认航点到达半径。"""
        self.arrival_radius = self._positive_float(
            arrival_radius, 'arrival_radius')
        self.trajectory = []
        self.current_state = MissionState.IDLE
        self.current_waypoint_index = 0
        self.current_position = None
        self.failsafe_reason = ''
        self.return_target = None
        self.px4_auto_land = False
        self.px4_disarmed = False

    @staticmethod
    def _positive_float(value, name):
        """将参数转换为有限正浮点数。"""
        try:
            result = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError('%s必须是正数' % name) from error
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError('%s必须是有限正数' % name)
        return result

    @staticmethod
    def _validate_trajectory(trajectory):
        """确认轨迹非空，且所有元素都是TrajectoryPoint。"""
        if not isinstance(trajectory, (tuple, list)) or not trajectory:
            raise ValueError('trajectory不能为空')
        if not all(isinstance(point, TrajectoryPoint) for point in trajectory):
            raise ValueError('trajectory中的元素必须是TrajectoryPoint')
        return list(trajectory)

    def load_trajectory(self, trajectory):
        """在IDLE状态加载并保存一条新轨迹。"""
        if self.current_state != MissionState.IDLE:
            raise RuntimeError('只能在IDLE状态加载轨迹')
        self.trajectory = self._validate_trajectory(trajectory)
        self.return_target = TrajectoryPoint(
            timestamp=0.0,
            x=0.0,
            y=0.0,
            z=1.2,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw=0.0,
            waypoint_id=len(self.trajectory),
        )
        self.current_waypoint_index = 0
        self.current_position = None
        self.failsafe_reason = ''
        self.px4_auto_land = False
        self.px4_disarmed = False

    def start(self):
        """启动已加载任务，使状态从IDLE进入TAKEOFF。"""
        if self.current_state != MissionState.IDLE:
            raise RuntimeError('任务只能从IDLE状态启动')
        if not self.trajectory:
            raise RuntimeError('启动前必须加载轨迹')
        self.current_waypoint_index = 0
        self.px4_auto_land = False
        self.px4_disarmed = False
        self.current_state = MissionState.TAKEOFF

    def update_position(self, x, y, z):
        """保存无人机当前位置。"""
        try:
            position = (float(x), float(y), float(z))
        except (TypeError, ValueError) as error:
            raise ValueError('位置必须是有限数值') from error
        if not all(math.isfinite(value) for value in position):
            raise ValueError('位置必须是有限数值')
        self.current_position = position

    def get_current_target(self):
        """返回当前状态需要到达的TrajectoryPoint，无目标时返回None。"""
        if not self.trajectory:
            return None

        if self.current_state in (MissionState.TAKEOFF, MissionState.EXECUTE):
            if self.current_waypoint_index < len(self.trajectory):
                return self.trajectory[self.current_waypoint_index]
            return None

        if self.current_state == MissionState.RETURN:
            return self.return_target

        return None

    def update_landing_status(self, auto_land, disarmed):
        """保存PX4是否处于自动降落模式以及是否已经锁桨。"""
        self.px4_auto_land = bool(auto_land)
        self.px4_disarmed = bool(disarmed)

    def distance_to_target(self):
        """计算当前位置到当前目标的三维欧氏距离。"""
        target = self.get_current_target()
        if self.current_position is None or target is None:
            return math.inf

        x, y, z = self.current_position
        return math.sqrt(
            (target.x - x) ** 2
            + (target.y - y) ** 2
            + (target.z - z) ** 2
        )

    def _target_reached(self):
        """判断当前位置是否进入目标到达半径。"""
        return self.distance_to_target() <= self.arrival_radius

    def step(self):
        """根据当前位置推进任务状态，并返回更新后的状态。"""
        if self.current_state == MissionState.TAKEOFF:
            # 首个轨迹点作为起飞阶段目标，到达后开始执行剩余轨迹。
            if self._target_reached():
                self.current_waypoint_index = 1
                if self.current_waypoint_index >= len(self.trajectory):
                    self.current_state = MissionState.RETURN
                else:
                    self.current_state = MissionState.EXECUTE

        elif self.current_state == MissionState.EXECUTE:
            # 提前切换到下一插值点，使位置目标保持在飞机前方。
            if self._target_reached():
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.trajectory):
                    self.current_state = MissionState.RETURN

        elif self.current_state == MissionState.RETURN:
            if self._target_reached():
                self.current_state = MissionState.LAND

        elif self.current_state == MissionState.LAND:
            if self.px4_auto_land and self.px4_disarmed:
                self.current_state = MissionState.COMPLETE

        return self.current_state

    def enter_failsafe(self, reason):
        """记录失败原因并进入FAILSAFE终止状态。"""
        self.failsafe_reason = str(reason)
        self.current_state = MissionState.FAILSAFE


def main():
    """运行任务启动、航点推进和完整状态流程三个简单测试。"""
    trajectory = [
        TrajectoryPoint(
            timestamp=0.0,
            x=0.25, y=0.25, z=1.2,
            vx=0.5, vy=0.0, vz=0.0,
            yaw=0.0, waypoint_id=0,
        ),
        TrajectoryPoint(
            timestamp=1.0,
            x=0.75, y=0.25, z=1.2,
            vx=0.5, vy=0.0, vz=0.0,
            yaw=0.0, waypoint_id=1,
        ),
        TrajectoryPoint(
            timestamp=2.0,
            x=1.25, y=0.25, z=1.2,
            vx=0.0, vy=0.0, vz=0.0,
            yaw=0.0, waypoint_id=2,
        ),
    ]
    manager = MissionManager()

    # 测试1：加载任务后启动，状态应从IDLE进入TAKEOFF。
    manager.load_trajectory(trajectory)
    assert manager.current_state == MissionState.IDLE
    manager.start()
    assert manager.current_state == MissionState.TAKEOFF
    assert manager.current_waypoint_index == 0
    print('测试1通过：IDLE → TAKEOFF')

    # 测试2：依次到达三个轨迹点，索引应自动推进。
    manager.update_position(0.25, 0.25, 1.2)
    manager.step()
    assert manager.current_state == MissionState.EXECUTE
    assert manager.current_waypoint_index == 1

    manager.update_position(0.75, 0.25, 1.2)
    manager.step()
    assert manager.current_waypoint_index == 2

    manager.update_position(1.25, 0.25, 1.2)
    manager.step()
    assert manager.current_state == MissionState.RETURN
    assert manager.current_waypoint_index == 3
    print('测试2通过：3个轨迹点自动推进，进入RETURN')

    # 测试3：返回固定起飞点，LAND保持到PX4自动降落并锁桨。
    return_target = manager.get_current_target()
    assert (
        return_target.x,
        return_target.y,
        return_target.z,
    ) == (0.0, 0.0, 1.2)
    manager.update_position(
        return_target.x, return_target.y, return_target.z)
    manager.step()
    assert manager.current_state == MissionState.LAND
    manager.step()
    assert manager.current_state == MissionState.LAND
    manager.update_landing_status(auto_land=True, disarmed=True)
    manager.step()
    assert manager.current_state == MissionState.COMPLETE
    print('测试3通过：RETURN → LAND保持 → PX4确认 → COMPLETE')


if __name__ == '__main__':
    main()
