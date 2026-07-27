"""ROS 2 camera node publishing the existing H7-compatible interface."""

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .camera_source import create_source
from .detector import INVALID_DETECTION, RedTargetDetector


def transform_frame(frame, rotation=0, flip_horizontal=False,
                    flip_vertical=False):
    """Apply installation orientation without changing acquisition backends."""
    rotations = {
        0: None,
        90: cv2.ROTATE_90_CLOCKWISE,
        180: cv2.ROTATE_180,
        270: cv2.ROTATE_90_COUNTERCLOCKWISE,
    }
    rotation = int(rotation)
    if rotation not in rotations:
        raise ValueError('rotation must be one of 0, 90, 180, 270')
    if rotations[rotation] is not None:
        frame = cv2.rotate(frame, rotations[rotation])
    if flip_horizontal and flip_vertical:
        return cv2.flip(frame, -1)
    if flip_horizontal:
        return cv2.flip(frame, 1)
    if flip_vertical:
        return cv2.flip(frame, 0)
    return frame


class PiCameraVisionNode(Node):
    """Acquire frames and publish target geometry with actual frame size."""

    def __init__(self):
        super().__init__('pi_camera_vision_node')
        defaults = {
            'source_type': 'video',
            'source': '',
            'device': 0,
            'loop': False,
            'width': 640,
            'height': 480,
            'fps': 15.0,
            'rotation': 0,
            'flip_horizontal': False,
            'flip_vertical': False,
            'min_area': 100.0,
            'morphology_kernel': 5,
            'red1_h_min': 0,
            'red1_h_max': 10,
            'red2_h_min': 170,
            'red2_h_max': 179,
            'saturation_min': 100,
            'value_min': 80,
            'detection_topic': '/vision/h7/detection',
            'display': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        fps = float(self.get_parameter('fps').value)
        if fps <= 0:
            raise ValueError('fps must be positive')
        self.display = bool(self.get_parameter('display').value)
        self.requested_width = float(self.get_parameter('width').value)
        self.requested_height = float(self.get_parameter('height').value)
        self.rotation = int(self.get_parameter('rotation').value)
        self.flip_horizontal = bool(
            self.get_parameter('flip_horizontal').value)
        self.flip_vertical = bool(
            self.get_parameter('flip_vertical').value)
        transform_frame(
            cv2.UMat(1, 1, cv2.CV_8UC3).get(), self.rotation,
            self.flip_horizontal, self.flip_vertical)
        self.source = create_source(
            source_type=self.get_parameter('source_type').value,
            source=self.get_parameter('source').value,
            device=self.get_parameter('device').value,
            loop=self.get_parameter('loop').value,
            width=self.get_parameter('width').value,
            height=self.get_parameter('height').value,
            fps=fps,
        )
        self.detector = RedTargetDetector(
            min_area=self.get_parameter('min_area').value,
            morphology_kernel=self.get_parameter('morphology_kernel').value,
            red1_h_min=self.get_parameter('red1_h_min').value,
            red1_h_max=self.get_parameter('red1_h_max').value,
            red2_h_min=self.get_parameter('red2_h_min').value,
            red2_h_max=self.get_parameter('red2_h_max').value,
            saturation_min=self.get_parameter('saturation_min').value,
            value_min=self.get_parameter('value_min').value,
        )
        self.detection_topic = self.get_parameter('detection_topic').value
        self.publisher = self.create_publisher(
            Float32MultiArray, self.detection_topic, 10)
        self.finished = False
        self.timer = self.create_timer(1.0 / fps, self.process_frame)
        self.get_logger().info(
            'Publishing %s; keep h7_bridge_node stopped'
            % self.detection_topic)

    def publish_detection(self, values):
        """Publish exactly nine normalized-geometry input fields."""
        message = Float32MultiArray()
        message.data = [float(value) for value in values]
        self.publisher.publish(message)

    def process_frame(self):
        """Process and publish one frame; EOF is not a camera failure."""
        if self.finished:
            return
        ok, frame = self.source.read()
        if not ok:
            self.finished = True
            invalid = list(INVALID_DETECTION)
            invalid[7:] = [self.requested_width, self.requested_height]
            self.publish_detection(invalid)
            self.get_logger().info(
                'Input ended or failed; published invalid detection')
            self.timer.cancel()
            rclpy.shutdown()
            return
        try:
            frame = transform_frame(
                frame, self.rotation, self.flip_horizontal,
                self.flip_vertical)
            values, annotated, _ = self.detector.detect(frame)
        except cv2.error as error:
            self.get_logger().warning('OpenCV frame error: %s' % error)
            values, annotated = list(INVALID_DETECTION), None
        self.publish_detection(values)
        if self.display and annotated is not None:
            cv2.imshow('pi_camera_vision', annotated)
            cv2.waitKey(1)

    def destroy_node(self):
        """Release camera resources before destroying the ROS node."""
        self.source.close()
        if self.display:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    """Run the ROS 2 vision node."""
    rclpy.init(args=args)
    node = None
    try:
        node = PiCameraVisionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
