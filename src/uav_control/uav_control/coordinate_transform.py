"""任务地图坐标系与PX4本地NED坐标系的纯Python转换逻辑。"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MapCoordinate:
    """任务地图坐标：x向东、y向北、z向上，单位为米。"""

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class PX4Coordinate:
    """PX4本地NED坐标：north向北、east向东、down向下，单位为米。"""

    north: float
    east: float
    down: float


class CoordinateTransformer:
    """执行任务地图坐标与PX4本地NED坐标之间的双向转换。"""

    def __init__(
            self,
            origin_north: float = 0.0,
            origin_east: float = 0.0,
            origin_down: float = 0.0,
            yaw_offset: float = 0.0,
    ) -> None:
        """保存地图原点在PX4 NED中的位置和偏航角修正量。"""
        values = (
            origin_north,
            origin_east,
            origin_down,
            yaw_offset,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('坐标原点和yaw_offset必须是有限数值')

        self.origin_north = float(origin_north)
        self.origin_east = float(origin_east)
        self.origin_down = float(origin_down)
        self.yaw_offset = float(yaw_offset)

    @staticmethod
    def _finite_coordinates(*values: float) -> tuple[float, ...]:
        """将输入转换为浮点数，并拒绝非有限坐标。"""
        try:
            coordinates = tuple(float(value) for value in values)
        except (TypeError, ValueError) as error:
            raise ValueError('坐标必须是有限数值') from error
        if not all(math.isfinite(value) for value in coordinates):
            raise ValueError('坐标必须是有限数值')
        return coordinates

    def map_to_px4_position(
            self, x: float, y: float, z: float) -> PX4Coordinate:
        """将东、北、上地图坐标转换为PX4北、东、下坐标。"""
        map_x, map_y, map_z = self._finite_coordinates(x, y, z)
        return PX4Coordinate(
            north=self.origin_north + map_y,
            east=self.origin_east + map_x,
            down=self.origin_down - map_z,
        )

    def px4_to_map_position(
            self,
            north: float,
            east: float,
            down: float,
    ) -> MapCoordinate:
        """将PX4北、东、下坐标反向转换为东、北、上地图坐标。"""
        px4_north, px4_east, px4_down = self._finite_coordinates(
            north, east, down)
        return MapCoordinate(
            x=px4_east - self.origin_east,
            y=px4_north - self.origin_north,
            z=self.origin_down - px4_down,
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """将角度归一化到[-pi, pi]范围。"""
        normalized_angle, = CoordinateTransformer._finite_coordinates(angle)
        return (normalized_angle + math.pi) % (2.0 * math.pi) - math.pi

    def map_to_px4_yaw(self, map_yaw: float) -> float:
        """将地图偏航角转换为PX4 NED偏航角并进行归一化。"""
        map_yaw_value, = self._finite_coordinates(map_yaw)
        px4_yaw = math.pi / 2.0 - map_yaw_value + self.yaw_offset
        return self.normalize_angle(px4_yaw)


def main() -> None:
    """运行位置正反转换和yaw转换测试。"""
    transformer = CoordinateTransformer()

    # 测试1：地图(东1、北2、上3)应转换为NED(北2、东1、下-3)。
    px4_position = transformer.map_to_px4_position(1.0, 2.0, 3.0)
    assert px4_position == PX4Coordinate(
        north=2.0,
        east=1.0,
        down=-3.0,
    )
    print(
        '位置转换通过：north=%.1f, east=%.1f, down=%.1f'
        % (px4_position.north, px4_position.east, px4_position.down)
    )

    # 测试2：反向转换应恢复原始地图坐标。
    map_position = transformer.px4_to_map_position(
        px4_position.north,
        px4_position.east,
        px4_position.down,
    )
    assert map_position == MapCoordinate(x=1.0, y=2.0, z=3.0)
    print(
        '反向转换通过：x=%.1f, y=%.1f, z=%.1f'
        % (map_position.x, map_position.y, map_position.z)
    )

    # 测试3：地图yaw为0时，PX4 yaw应为pi/2。
    px4_yaw = transformer.map_to_px4_yaw(0.0)
    assert math.isclose(px4_yaw, math.pi / 2.0)
    print('yaw转换通过：map_yaw=0.0, px4_yaw=%.6f' % px4_yaw)


if __name__ == '__main__':
    main()
