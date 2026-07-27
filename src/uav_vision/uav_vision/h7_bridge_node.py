"""通过串口接收H7Plus目标检测数据并发布到ROS 2。"""

import math

import rclpy
from rclpy.node import Node
import serial
from serial import SerialException
from std_msgs.msg import Float32MultiArray

from .detection import validate_detection


class H7BridgeNode(Node):
    """读取并校验H7Plus串口协议。"""

    def __init__(self):
        super().__init__('h7_bridge_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter(
            'detection_topic', '/vision/h7/detection')
        self.declare_parameter('image_width', 320.0)
        self.declare_parameter('image_height', 240.0)
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.image_width = float(self.get_parameter('image_width').value)
        self.image_height = float(self.get_parameter('image_height').value)
        validate_detection([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            self.image_width, self.image_height])

        self.publisher = self.create_publisher(
            Float32MultiArray, self.detection_topic, 10)
        self.serial_port = None

        # 短超时保证串口无数据时不会长期阻塞ROS 2执行器。
        self.timer = self.create_timer(0.01, self.read_serial)
        self.reconnect_timer = self.create_timer(2.0, self.open_serial)
        self.open_serial()

    def warn_throttled(self, message):
        """对重复错误限频，避免异常串口数据刷屏。"""
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def open_serial(self):
        """打开串口；设备暂时缺失时等待下次重试。"""
        if self.serial_port is not None and self.serial_port.is_open:
            return
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.005,
            )
            self.get_logger().info(
                f'H7Plus串口已连接：{self.port}，{self.baudrate} baud')
        except (SerialException, OSError, ValueError) as error:
            self.serial_port = None
            self.warn_throttled(f'无法打开H7Plus串口：{error}')

    @staticmethod
    def parse_line(line, image_width=320.0, image_height=240.0):
        """校验一行协议并返回按发布顺序排列的数据。"""
        fields = line.strip().split(',')
        if len(fields) != 8:
            raise ValueError(f'字段数量应为8，实际为{len(fields)}')
        if fields[0] != 'TARGET':
            raise ValueError('消息类型必须为TARGET')

        try:
            valid = int(fields[1])
            values = [float(value) for value in fields[2:]]
        except ValueError as error:
            raise ValueError('字段包含非法数值') from error

        if valid not in (0, 1):
            raise ValueError('valid必须为0或1')
        if not all(math.isfinite(value) for value in values):
            raise ValueError('数值必须为有限值')

        cx, cy, width, height, area, confidence = values
        if cx < 0.0 or cy < 0.0:
            raise ValueError('cx和cy不能为负数')
        if width < 0.0 or height < 0.0 or area < 0.0:
            raise ValueError('width、height和area不能为负数')
        if not 0.0 <= confidence <= 100.0:
            raise ValueError('confidence必须在0到100之间')

        return validate_detection([
            float(valid), cx, cy, width, height, area, confidence,
            image_width, image_height])

    def read_serial(self):
        """读取当前可用数据，错误行不会终止节点。"""
        if self.serial_port is None or not self.serial_port.is_open:
            return
        try:
            # 每次定时最多处理100行，避免大量积压数据饿死执行器。
            for _ in range(100):
                raw_line = self.serial_port.readline()
                if not raw_line:
                    break
                try:
                    line = raw_line.decode('utf-8').strip()
                    data = self.parse_line(
                        line, self.image_width, self.image_height)
                except (UnicodeDecodeError, ValueError) as error:
                    self.warn_throttled(f'忽略无效H7Plus数据：{error}')
                    continue

                message = Float32MultiArray()
                message.data = data
                self.publisher.publish(message)
        except (SerialException, OSError) as error:
            self.warn_throttled(f'H7Plus串口读取失败：{error}')
            try:
                self.serial_port.close()
            except (SerialException, OSError):
                pass
            self.serial_port = None

    def destroy_node(self):
        """节点退出时释放串口。"""
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = H7BridgeNode()
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
