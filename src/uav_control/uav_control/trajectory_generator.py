"""将覆盖规划航点转换为连续位置轨迹的纯Python逻辑。"""

from dataclasses import dataclass
import math
from typing import Dict, List, Sequence, Tuple


@dataclass
class TrajectoryPoint:
    """表示带时间、位置、速度和航点编号的一个轨迹采样点。"""

    timestamp: float
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    yaw: float
    waypoint_id: int


class TrajectoryGenerator:
    """以恒定速度在航点间插值，生成position模式连续轨迹。"""

    def __init__(self, speed=0.5, sample_period=0.1):
        """设置默认飞行速度和轨迹采样周期。"""
        self.speed = self._positive_float(speed, 'speed')
        self.sample_period = self._positive_float(
            sample_period, 'sample_period')

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
    def _waypoint_position(
            waypoint: Dict[str, object]) -> Tuple[float, float, float]:
        """读取并校验航点中的x、y、z坐标。"""
        if not isinstance(waypoint, dict):
            raise ValueError('waypoint必须是字典')
        try:
            position = tuple(
                float(waypoint[name]) for name in ('x', 'y', 'z'))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError('waypoint必须包含有限数值x、y、z') from error
        if not all(math.isfinite(value) for value in position):
            raise ValueError('waypoint坐标必须是有限数值')
        return position

    @classmethod
    def _normalized_positions(
            cls,
            waypoints: Sequence[Dict[str, object]],
    ) -> List[Tuple[int, Tuple[float, float, float]]]:
        """读取航点并合并连续重复位置，保证生成时间戳严格递增。"""
        normalized = []
        for waypoint_id, waypoint in enumerate(waypoints):
            position = cls._waypoint_position(waypoint)
            if normalized and position == normalized[-1][1]:
                # 保留重复位置中最后一个输入航点的编号。
                normalized[-1] = (waypoint_id, position)
            else:
                normalized.append((waypoint_id, position))
        return normalized

    @staticmethod
    def _position_tuple(position):
        """将字典、轨迹点或三元素序列转换为位置元组。"""
        if isinstance(position, TrajectoryPoint):
            values = (position.x, position.y, position.z)
        elif isinstance(position, dict):
            try:
                values = tuple(
                    float(position[name]) for name in ('x', 'y', 'z'))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError('位置字典必须包含x、y、z') from error
        elif isinstance(position, (tuple, list)) and len(position) == 3:
            try:
                values = tuple(float(value) for value in position)
            except (TypeError, ValueError) as error:
                raise ValueError('位置必须包含三个有限数值') from error
        else:
            raise ValueError('位置必须是轨迹点、字典或三元素序列')
        if not all(math.isfinite(value) for value in values):
            raise ValueError('位置必须包含三个有限数值')
        return values

    @classmethod
    def distance_to_waypoint(cls, position, waypoint):
        """计算当前位置到目标航点的三维欧氏距离。"""
        current_x, current_y, current_z = cls._position_tuple(position)
        target_x, target_y, target_z = cls._waypoint_position(waypoint)
        return math.sqrt(
            (target_x - current_x) ** 2
            + (target_y - current_y) ** 2
            + (target_z - current_z) ** 2
        )

    @staticmethod
    def _segment_values(start, end, speed, previous_yaw):
        """计算航段距离、单位速度和水平航向角。"""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance == 0.0:
            return distance, (0.0, 0.0, 0.0), previous_yaw

        scale = speed / distance
        velocity = (dx * scale, dy * scale, dz * scale)
        yaw = math.atan2(dy, dx) if dx != 0.0 or dy != 0.0 else previous_yaw
        return distance, velocity, yaw

    def generate(
            self,
            waypoints: Sequence[Dict[str, object]],
            mode: str = 'position',
    ) -> List[TrajectoryPoint]:
        """将航点列表转换为按时间递增的连续轨迹点列表。"""
        if mode != 'position':
            raise ValueError('当前仅支持position模式')
        if not isinstance(waypoints, (tuple, list)) or not waypoints:
            raise ValueError('waypoints不能为空')

        normalized = self._normalized_positions(waypoints)
        waypoint_ids = [item[0] for item in normalized]
        positions = [item[1] for item in normalized]
        if len(positions) == 1:
            x, y, z = positions[0]
            return [TrajectoryPoint(
                timestamp=0.0,
                x=x,
                y=y,
                z=z,
                vx=0.0,
                vy=0.0,
                vz=0.0,
                yaw=0.0,
                waypoint_id=waypoint_ids[0],
            )]

        first_distance, first_velocity, yaw = self._segment_values(
            positions[0], positions[1], self.speed, 0.0)
        del first_distance
        trajectory = [TrajectoryPoint(
            timestamp=0.0,
            x=positions[0][0],
            y=positions[0][1],
            z=positions[0][2],
            vx=first_velocity[0],
            vy=first_velocity[1],
            vz=first_velocity[2],
            yaw=yaw,
            waypoint_id=waypoint_ids[0],
        )]

        elapsed = 0.0
        for position_index in range(1, len(positions)):
            waypoint_id = waypoint_ids[position_index]
            start = positions[position_index - 1]
            end = positions[position_index]
            distance, velocity, yaw = self._segment_values(
                start, end, self.speed, yaw)

            duration = distance / self.speed
            sample_count = max(1, math.ceil(duration / self.sample_period))
            for sample_index in range(1, sample_count + 1):
                segment_time = min(
                    sample_index * self.sample_period, duration)
                ratio = segment_time / duration
                trajectory.append(TrajectoryPoint(
                    timestamp=elapsed + segment_time,
                    x=start[0] + (end[0] - start[0]) * ratio,
                    y=start[1] + (end[1] - start[1]) * ratio,
                    z=start[2] + (end[2] - start[2]) * ratio,
                    vx=velocity[0],
                    vy=velocity[1],
                    vz=velocity[2],
                    yaw=yaw,
                    waypoint_id=waypoint_id,
                ))
            elapsed += duration

        # 最终航点到达后速度归零，位置和累计时间保持不变。
        final = trajectory[-1]
        trajectory[-1] = TrajectoryPoint(
            timestamp=final.timestamp,
            x=final.x,
            y=final.y,
            z=final.z,
            vx=0.0,
            vy=0.0,
            vz=0.0,
            yaw=final.yaw,
            waypoint_id=final.waypoint_id,
        )
        return trajectory

    def generate_return_to_start(self, current_position, waypoints):
        """从当前位置生成返回首个航点的连续轨迹。"""
        if not isinstance(waypoints, (tuple, list)) or not waypoints:
            raise ValueError('waypoints不能为空')
        current_x, current_y, current_z = self._position_tuple(
            current_position)
        start_x, start_y, start_z = self._waypoint_position(waypoints[0])
        return self.generate([
            {
                'cell_code': 'CURRENT',
                'x': current_x,
                'y': current_y,
                'z': current_z,
            },
            {
                'cell_code': waypoints[0].get('cell_code', 'START'),
                'x': start_x,
                'y': start_y,
                'z': start_z,
            },
        ])


def main():
    """运行覆盖规划接口和轨迹输出约束测试。"""
    waypoints = [
        {'cell_code': 'A1B1', 'x': 0.25, 'y': 0.25, 'z': 1.2},
        {'cell_code': 'A3B1', 'x': 1.25, 'y': 0.25, 'z': 1.2},
        {'cell_code': 'A3B3', 'x': 1.25, 'y': 1.25, 'z': 1.2},
    ]
    generator = TrajectoryGenerator(speed=0.5, sample_period=0.1)

    # 测试1：直接接受coverage_planner输出格式的航点字典。
    trajectory = generator.generate(waypoints)
    assert all(isinstance(point, TrajectoryPoint) for point in trajectory)
    print('测试1通过：兼容coverage_planner航点格式')

    # 测试2：两个1m航段应按0.1s周期生成41个轨迹点。
    assert len(trajectory) == 41
    assert trajectory[0].waypoint_id == 0
    assert trajectory[-1].waypoint_id == 2
    print('测试2通过：TrajectoryPoint数量=%d' % len(trajectory))

    # 测试3：所有相邻轨迹点的时间戳必须严格递增。
    assert all(
        current.timestamp < following.timestamp
        for current, following in zip(trajectory, trajectory[1:])
    )
    assert math.isclose(trajectory[-1].timestamp, 4.0)
    print('测试3通过：timestamp严格递增，总时间=%.2fs'
          % trajectory[-1].timestamp)

    # 测试4：到达最后一个轨迹点后，三轴速度必须归零。
    assert trajectory[-1].vx == 0.0
    assert trajectory[-1].vy == 0.0
    assert trajectory[-1].vz == 0.0
    print('测试4通过：最后一点速度为0')

    # 测试5：单航点输入生成一个静止轨迹点，不进入航段插值。
    single_trajectory = generator.generate([waypoints[0]])
    assert len(single_trajectory) == 1
    assert single_trajectory[0].timestamp == 0.0
    assert (
        single_trajectory[0].vx,
        single_trajectory[0].vy,
        single_trajectory[0].vz,
    ) == (0.0, 0.0, 0.0)
    print('测试5通过：单航点输入生成1个静止轨迹点')


if __name__ == '__main__':
    main()
