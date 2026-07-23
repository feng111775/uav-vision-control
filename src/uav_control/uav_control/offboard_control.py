import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from px4_msgs.msg import VehicleCommand

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint


class OffboardControl(Node):
    """持续发送Offboard心跳和固定位置目标，但暂不切换模式和解锁。"""

    def __init__(self):
        super().__init__('offboard_control')

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 告诉PX4我们使用哪一种控制方式。
        self.offboard_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            px4_qos,
        )

        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            px4_qos,
        )

        self.setpoint_counter = 0

        # 向PX4发送期望位置。
        self.trajectory_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            px4_qos,
        )

        # 每0.1秒执行一次，即10Hz。
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            'Offboard设定值节点已启动：仅发送目标点，不切换模式，不解锁'
        )

    def get_timestamp(self):
        """生成PX4消息需要的微秒时间戳。"""
        return self.get_clock().now().nanoseconds // 1000

    def publish_offboard_control_mode(self):
        """声明采用位置控制模式。"""
        message = OffboardControlMode()

        message.timestamp = self.get_timestamp()
        message.position = True
        message.velocity = False
        message.acceleration = False
        message.attitude = False
        message.body_rate = False

        self.offboard_mode_publisher.publish(message)

    def publish_trajectory_setpoint(self):
        """发送起点上方2米的位置目标。"""
        message = TrajectorySetpoint()

        message.timestamp = self.get_timestamp()

        # PX4采用NED坐标系，Z为负数表示向上。
        message.position = [0.0, 0.0, -2.0]

        # 暂时不控制速度和加速度。
        nan = float('nan')
        message.velocity = [nan, nan, nan]
        message.acceleration = [nan, nan, nan]

        # 机头朝向0弧度。
        message.yaw = 0.0
        message.yawspeed = nan

        self.trajectory_publisher.publish(message)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """向PX4发送飞控命令。"""
        message = VehicleCommand()

        message.timestamp = self.get_timestamp()
        message.param1 = param1
        message.param2 = param2
        message.command = command

        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1

        message.from_external = True

        self.vehicle_command_publisher.publish(message)
    
    def timer_callback(self):
        """持续发送控制设定值，并依次切换模式和解锁。"""
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # 运行约1秒后请求进入Offboard模式。
        if self.setpoint_counter == 10:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
        )
        self.get_logger().info('已请求切换到Offboard模式')

        # 再等待约1秒，然后发送解锁命令。
        if self.setpoint_counter == 20:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
        )
            self.get_logger().info('已发送解锁命令，仿真无人机将飞往2米高度')

        if self.setpoint_counter < 151:
            self.setpoint_counter += 1

        # 节点运行约15秒后，命令PX4自动降落。
        if self.setpoint_counter == 150:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND,
        )
            self.get_logger().info('已发送自动降落命令')


def main(args=None):
    rclpy.init(args=args)

    node = OffboardControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到停止命令，正在关闭节点')
    finally:
        node.destroy_node()

    # 只有ROS 2尚未关闭时才执行关闭，避免重复调用。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()