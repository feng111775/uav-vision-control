"""Select one fresh resolution-aware detection from two cameras."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

from .detection import INVALID_DETECTION
from .detection import normalized_geometry
from .detection import validate_detection



class CameraSelector:
    """ROS-independent dual-camera selection state machine."""

    SEARCH = 'SEARCH'
    DOWN_ACQUIRE = 'DOWN_ACQUIRE'
    ALIGN = 'ALIGN'

    def __init__(self, mode='auto', front_confirm_frames=3,
                 front_area_ratio_threshold=0.1,
                 front_width_ratio_threshold=0.35,
                 front_height_ratio_threshold=0.35, down_confirm_frames=3,
                 down_hold_frames=2, down_lost_frames=5,
                 source_timeout=0.3, switch_cooldown=2.0):
        """Validate thresholds and initialize selection state."""
        if mode not in ('front', 'down', 'auto'):
            raise ValueError('mode must be front, down, or auto')
        counts = (front_confirm_frames, down_confirm_frames,
                  down_hold_frames, down_lost_frames)
        if any(int(value) < 1 for value in counts):
            raise ValueError('frame thresholds must be at least 1')
        numeric = (
            front_area_ratio_threshold, front_width_ratio_threshold,
            front_height_ratio_threshold, source_timeout, switch_cooldown)
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError('selector thresholds must be finite')
        ratios = numeric[:3]
        if any(value < 0.0 or value > 1.0 for value in ratios):
            raise ValueError('front switch thresholds cannot be negative')
        if source_timeout <= 0.0:
            raise ValueError('source_timeout must be positive')
        if switch_cooldown < 0.0:
            raise ValueError('switch_cooldown cannot be negative')
        if down_hold_frames >= down_lost_frames:
            raise ValueError('down_hold_frames must be below down_lost_frames')

        self.mode = mode
        self.front_confirm_frames = int(front_confirm_frames)
        self.front_area_ratio_threshold = float(
            front_area_ratio_threshold)
        self.front_width_ratio_threshold = float(
            front_width_ratio_threshold)
        self.front_height_ratio_threshold = float(
            front_height_ratio_threshold)
        self.down_confirm_frames = int(down_confirm_frames)
        self.down_hold_frames = int(down_hold_frames)
        self.down_lost_frames = int(down_lost_frames)
        self.source_timeout = float(source_timeout)
        self.switch_cooldown = float(switch_cooldown)

        self.state = self.SEARCH
        self.selected_camera = mode if mode in ('front', 'down') else 'front'
        self.front_detection = list(INVALID_DETECTION)
        self.down_detection = list(INVALID_DETECTION)
        self.last_valid_down = list(INVALID_DETECTION)
        self.front_time = None
        self.down_time = None
        self.front_close_count = 0
        self.down_valid_count = 0
        self.down_lost_count = 0
        self.down_selected_time = None
        self.last_down_failure_time = -math.inf

    @staticmethod
    def validate(values):
        """Validate one nine-field resolution-aware detection."""
        return validate_detection(values)

    @staticmethod
    def is_valid(detection):
        """Return whether a normalized detection contains a target."""
        return detection[0] == 1.0

    def front_is_close(self, detection):
        """Apply resolution-independent area and target-size thresholds."""
        if not self.is_valid(detection):
            return False
        _, _, area_ratio, width_ratio, height_ratio = normalized_geometry(
            detection)
        return (
            area_ratio >= self.front_area_ratio_threshold
            or width_ratio >= self.front_width_ratio_threshold
            or height_ratio >= self.front_height_ratio_threshold)

    def update_front(self, values, now_seconds):
        """Record a front frame and advance front confirmation."""
        self.front_detection = self.validate(values)
        self.front_time = float(now_seconds)
        if self.front_is_close(self.front_detection):
            self.front_close_count += 1
        else:
            self.front_close_count = 0

        if (self.mode == 'auto' and self.state == self.SEARCH
                and self.front_close_count >= self.front_confirm_frames
                and float(now_seconds) - self.last_down_failure_time
                >= self.switch_cooldown):
            self.state = self.DOWN_ACQUIRE
            # 进入获取阶段后仍用前视检测缓慢接近，直到下视目标经过连续
            # 帧确认。这样不会把一条尚未有效的下视检测送入控制链。
            self.selected_camera = 'front'
            self.down_valid_count = 0
            self.down_lost_count = 0
            self.down_selected_time = float(now_seconds)

    def update_down(self, values, now_seconds):
        """Record a down frame and advance acquire/loss hysteresis."""
        self.down_detection = self.validate(values)
        self.down_time = float(now_seconds)
        if self.is_valid(self.down_detection):
            self.last_valid_down = list(self.down_detection)
            self.down_valid_count += 1
            self.down_lost_count = 0
            if (self.mode == 'auto'
                    and self.state == self.DOWN_ACQUIRE
                    and self.down_valid_count >= self.down_confirm_frames):
                self.state = self.ALIGN
                self.selected_camera = 'down'
        else:
            self.down_valid_count = 0
            self.down_lost_count += 1
            if (self.mode == 'auto'
                    and self.state == self.ALIGN
                    and self.down_lost_count >= self.down_lost_frames):
                self.state = self.SEARCH
                self.selected_camera = 'front'
                self.front_close_count = 0
                self.last_down_failure_time = float(now_seconds)

    def fresh(self, camera, now_seconds):
        """Return whether the selected source has a recent message."""
        message_time = self.front_time if camera == 'front' else self.down_time
        return (
            message_time is not None
            and float(now_seconds) - message_time <= self.source_timeout
        )

    def selected(self, now_seconds):
        """Return (camera, state, detection), invalidating stale data."""
        if (self.mode == 'auto'
                and self.selected_camera == 'down'
                and not self.fresh('down', now_seconds)
                and self.down_selected_time is not None
                and float(now_seconds) - self.down_selected_time
                > self.source_timeout):
            self.state = self.SEARCH
            self.selected_camera = 'front'
            self.front_close_count = 0
            self.last_down_failure_time = float(now_seconds)

        if self.mode == 'front':
            camera = 'front'
        elif self.mode == 'down':
            camera = 'down'
        else:
            camera = self.selected_camera

        if not self.fresh(camera, now_seconds):
            return camera, self.state, list(INVALID_DETECTION)
        detection = (
            self.front_detection if camera == 'front'
            else self.down_detection)
        if self.is_valid(detection):
            return camera, self.state, list(detection)
        if (camera == 'down'
                and self.down_lost_count <= self.down_hold_frames
                and self.is_valid(self.last_valid_down)):
            return camera, self.state, list(self.last_valid_down)
        return camera, self.state, list(INVALID_DETECTION)


class CameraSelectorNode(Node):
    """Publish only the selected fresh detection for downstream control."""

    def __init__(self):
        """Create subscriptions, selected output, and fixed-rate timer."""
        super().__init__('camera_selector_node')
        defaults = {
            'mode': 'auto',
            'front_detection_topic': '/vision/front/detection',
            'down_detection_topic': '/vision/down/detection',
            'selected_detection_topic': '/vision/selected_detection',
            'selected_camera_topic': '/vision/selected_camera',
            'front_confirm_frames': 3,
            'front_area_ratio_threshold': 0.1,
            'front_width_ratio_threshold': 0.35,
            'front_height_ratio_threshold': 0.35,
            'down_confirm_frames': 3,
            'down_hold_frames': 2,
            'down_lost_frames': 5,
            'source_timeout': 0.3,
            'switch_cooldown': 2.0,
            'publish_rate': 20.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.selector = CameraSelector(**{
            name: self.get_parameter(name).value
            for name in (
                'mode', 'front_confirm_frames',
                'front_area_ratio_threshold',
                'front_width_ratio_threshold',
                'front_height_ratio_threshold', 'down_confirm_frames',
                'down_hold_frames', 'down_lost_frames', 'source_timeout',
                'switch_cooldown')
        })
        publish_rate = float(self.get_parameter('publish_rate').value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive and finite')

        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('selected_detection_topic').value, 10)
        self.camera_publisher = self.create_publisher(
            String,
            self.get_parameter('selected_camera_topic').value, 10)
        self.front_subscription = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('front_detection_topic').value,
            self.front_callback, 10)
        self.down_subscription = self.create_subscription(
            Float32MultiArray,
            self.get_parameter('down_detection_topic').value,
            self.down_callback, 10)
        self.timer = self.create_timer(
            1.0 / publish_rate, self.publish_selected)
        self.last_reported = None
        self.report_state(force=True)

    def now_seconds(self):
        """Return ROS time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def report_state(self, force=False):
        """Log state/camera transitions without flooding output."""
        current = (self.selector.state, self.selector.selected_camera)
        if force or current != self.last_reported:
            self.get_logger().info(
                'camera selector: mode=%s state=%s camera=%s'
                % (self.selector.mode, current[0], current[1]))
            self.last_reported = current

    def update(self, camera, values):
        """Validate one source without allowing malformed stale output."""
        try:
            if camera == 'front':
                self.selector.update_front(values, self.now_seconds())
            else:
                self.selector.update_down(values, self.now_seconds())
        except (TypeError, ValueError) as error:
            self.get_logger().warning(
                'invalid %s detection: %s' % (camera, error),
                throttle_duration_sec=5.0)
            invalid = list(INVALID_DETECTION)
            if camera == 'front':
                self.selector.update_front(invalid, self.now_seconds())
            else:
                self.selector.update_down(invalid, self.now_seconds())
        self.report_state()

    def front_callback(self, message):
        """Receive one front detection."""
        self.update('front', message.data)

    def down_callback(self, message):
        """Receive one down detection."""
        self.update('down', message.data)

    def publish_selected(self):
        """Publish exactly one selected detection at a fixed rate."""
        camera, state, detection = self.selector.selected(self.now_seconds())
        self.report_state()
        output = Float32MultiArray()
        output.data = detection
        camera_output = String()
        camera_output.data = camera
        self.camera_publisher.publish(camera_output)
        self.publisher.publish(output)
        if detection[0] == 0.0:
            self.get_logger().warning(
                'selected detection invalid: state=%s camera=%s'
                % (state, camera), throttle_duration_sec=5.0)


def main(args=None):
    """Run the dual-camera selector."""
    rclpy.init(args=args)
    node = CameraSelectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
