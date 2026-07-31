"""Read-only V2 observation adapter with explicit frame and timeout semantics."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from uav_interfaces.msg import TargetObservation, LandingError, VisionHealth, MissionVisionState

from .vision_protocol import parse_target, ClockMapper, Confirmation, SequenceTracker

NAN = float('nan')
STALE_TIMEOUT_SEC = 0.30


class VisionInterfaceNode(Node):
    def __init__(self):
        super().__init__('vision_interface_node')
        best = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tracked = self.create_publisher(TargetObservation, '/vision/target/tracked', best)
        self.landing = self.create_publisher(LandingError, '/vision/landing_error', best)
        self.health_pub = self.create_publisher(VisionHealth, '/vision/health', reliable)
        self.create_subscription(String, '/vision/internal/h7/raw', self.raw_cb, 20)
        self.create_subscription(String, '/vision/h7/status', self.status_cb, 10)
        self.create_subscription(MissionVisionState, '/uav/mission/state', self.state_cb, state_qos)
        self.mapper = ClockMapper(); self.sequence = SequenceTracker(); self.confirm = Confirmation()
        self.mode = 'SEARCH'; self.last = None; self.last_new = None
        self.connected = False; self.stale_published = False
        self.protocol_error_count = 0; self.frames = 0; self.valid_frames = 0
        self.confirmed_frames = 0; self.processing = []; self.received = []
        self.publish_times = []; self.stats = self._new_stats()
        self.create_timer(0.05, self._check_stale)
        self.create_timer(0.2, self.publish_health)

    def _new_stats(self):
        return {
            'input_fps': 0.0, 'new_frame_fps': 0.0, 'valid_detection_fps': 0.0,
            'publish_fps': 0.0, 'processing_p50_ms': NAN,
            'processing_p95_ms': NAN, 'processing_p99_ms': NAN,
            'serial_transport_p95_ms': NAN, 'end_to_end_p95_ms': NAN,
            'end_to_end_p99_ms': NAN, 'last_measurement_age_ms': NAN,
            'duplicate_sequence_count': 0, 'dropped_frame_count': 0,
            'out_of_order_sequence_count': 0, 'protocol_error_count': 0,
            'reconnect_count': 0,
        }

    def state_cb(self, msg):
        self.mode = {0: 'MISSION_IDLE', 1: 'SEARCH', 2: 'FOLLOW', 3: 'DROP_ALIGN'}.get(msg.state, 'SEARCH')

    def status_cb(self, msg):
        status = str(msg.data)
        if status == 'PROTOCOL_ERROR':
            self.protocol_error_count += 1
            self.stats['protocol_error_count'] = self.protocol_error_count
            return
        if status in ('DISCONNECTED', 'RECOVERED', 'CONNECTED'):
            self.mapper.reset(); self.sequence.reset(); self.confirm = Confirmation()
        if status == 'DISCONNECTED':
            self.connected = False
            self._publish_stale_once()
        elif status in ('RECOVERED', 'CONNECTED'):
            if status == 'RECOVERED': self.stats['reconnect_count'] += 1
            self.connected = True; self.stale_published = True

    def raw_cb(self, msg):
        try:
            item = parse_target(msg.data)
        except (TypeError, ValueError):
            self.protocol_error_count += 1
            self.stats['protocol_error_count'] = self.protocol_error_count
            return
        if item['version'] != 2 or item.get('sequence') is None:
            if not item.get('version') == 2:  # legacy protocol is disabled on formal V2 path
                self.protocol_error_count += 1
            return
        accepted, reason = self.sequence.accept(item['sequence'])
        self.stats['duplicate_sequence_count'] = self.sequence.duplicate_count
        self.stats['dropped_frame_count'] = self.sequence.dropped_count
        self.stats['out_of_order_sequence_count'] = self.sequence.out_of_order_count
        if not accepted:
            return
        if reason in ('restart', 'wrap'):
            self.mapper.reset(); self.confirm = Confirmation()
        now_ns = self.get_clock().now().nanoseconds
        now = time.monotonic(); self.received.append(now); self.received = self.received[-400:]
        item['raw_confidence'] = float(item['confidence'])
        item['confidence'] = max(0.0, min(1.0, item['confidence'] / 100.0))
        finite = all(math.isfinite(item[key]) for key in ('cx', 'cy', 'outer', 'inner', 'angle'))
        measurement_valid = bool(item['valid'] and finite)
        confirmed, count = False, 0
        if measurement_valid:
            _, confirmed, count = self.confirm.ingest({**item, 'confidence': item['raw_confidence']})
        else:
            self.confirm.count = 0
        self.frames += 1; self.valid_frames += int(measurement_valid)
        self.confirmed_frames += int(confirmed)
        self.last_new = now; self.stale_published = False; self.connected = True
        item.update(measurement_valid=measurement_valid, confirmed=confirmed,
                    count=count, received_ns=now_ns)
        stamp_ns, synced = self.mapper.update(item['ticks'], now_ns)
        if stamp_ns > now_ns:
            stamp_ns = now_ns; synced = False
        item['capture_stamp_ns'] = stamp_ns; item['capture_stamp_valid'] = bool(synced)
        self.processing.append(item['processing_us'] / 1000.0); self.processing = self.processing[-400:]
        self.last = item; self.publish(item, stale=False)

    def _publish_stale_once(self):
        if self.stale_published: return
        self.stale_published = True
        item = self.last or {'sequence': 0, 'capture_stamp_ns': self.get_clock().now().nanoseconds}
        self.publish(item, stale=True)

    def _check_stale(self):
        if self.last_new is None or time.monotonic() - self.last_new > STALE_TIMEOUT_SEC:
            self._publish_stale_once()

    def publish(self, item, stale=False):
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = now_ns if stale else item.get('capture_stamp_ns', now_ns)
        m = TargetObservation(); m.header.stamp = self._stamp(stamp_ns)
        m.capture_stamp_valid = bool(item.get('capture_stamp_valid', False) and not stale)
        m.target_id = 1; m.frame_sequence = int(item.get('sequence', 0)) & 0xffffffff
        valid = bool(item.get('measurement_valid', False) and not stale)
        m.detected = valid; m.confirmed = bool(item.get('confirmed', False) and valid)
        m.measurement_valid = valid; m.predicted = False
        m.consecutive_valid_frames = int(item.get('count', 0)) if valid else 0
        m.confidence = float(item.get('confidence', 0.0)) if valid else 0.0
        m.image_width = 320; m.image_height = 240
        if valid:
            m.center_x_px = item['cx']; m.center_y_px = item['cy']
            m.error_x_norm = (item['cx'] - 160.0) / 160.0
            m.error_y_norm = (item['cy'] - 120.0) / 120.0
            m.outer_diameter_px = item['outer']; m.inner_diameter_px = item['inner']
            m.target_angle_rad = item['angle']; m.platform_center_valid = True
            m.platform_center_x_px = item['cx']; m.platform_center_y_px = item['cy']
        else:
            for name in ('center_x_px', 'center_y_px', 'error_x_norm', 'error_y_norm',
                         'outer_diameter_px', 'inner_diameter_px', 'target_angle_rad',
                         'platform_center_x_px', 'platform_center_y_px', 'forward_m', 'left_m'):
                setattr(m, name, NAN)
            m.platform_center_valid = False
        m.metric_valid = False; m.forward_m = NAN; m.left_m = NAN
        self.tracked.publish(m)
        e = LandingError(); e.header = m.header; e.capture_stamp_valid = m.capture_stamp_valid
        e.frame_sequence = m.frame_sequence; e.confidence = m.confidence
        e.error_x_norm = m.error_x_norm; e.error_y_norm = m.error_y_norm; e.metric_valid = False
        e.forward_m = NAN; e.left_m = NAN
        e.valid = bool(m.detected and m.confirmed and m.measurement_valid and not m.predicted)
        self.landing.publish(e)
        self.publish_times.append(time.monotonic()); self.publish_times = self.publish_times[-400:]

    def _stamp(self, ns):
        from builtin_interfaces.msg import Time
        value = Time(); value.sec = int(ns // 1000000000); value.nanosec = int(ns % 1000000000)
        return value

    def publish_health(self):
        now = time.monotonic(); age = (now - self.last_new) * 1000.0 if self.last_new else NAN
        input_fps = self._rate(self.received); new_fps = input_fps
        valid_fps = self.valid_frames / max(now - (self.received[0] if self.received else now), 1e-9)
        publish_fps = self._rate(self.publish_times)
        self.stats.update(input_fps=input_fps, new_frame_fps=new_fps,
                          valid_detection_fps=valid_fps, publish_fps=publish_fps,
                          processing_p50_ms=self._pct(self.processing, .50),
                          processing_p95_ms=self._pct(self.processing, .95),
                          processing_p99_ms=self._pct(self.processing, .99),
                          last_measurement_age_ms=max(0.0, age) if math.isfinite(age) else NAN)
        h = VisionHealth(); h.header.stamp = self.get_clock().now().to_msg()
        h.camera_open = self.connected; h.frames_received = self.frames > 0
        h.algorithm_alive = self.last_new is not None and age < 300.0
        h.protocol_ok = self.protocol_error_count == 0
        h.calibration_loaded = False
        h.ready_for_mission = bool(h.camera_open and h.frames_received and h.algorithm_alive and
                                   h.protocol_ok and new_fps >= 20.0 and age < 300.0 and
                                   self.protocol_error_count == 0)
        h.ready_for_closed_loop = False
        h.performance_gate_passed = bool(new_fps >= 20.0 and h.protocol_ok)
        h.input_fps = input_fps; h.new_frame_fps = new_fps; h.valid_detection_fps = valid_fps
        h.publish_fps = publish_fps; h.processing_p50_ms = self.stats['processing_p50_ms']
        h.processing_p95_ms = self.stats['processing_p95_ms']; h.serial_transport_p95_ms = NAN
        h.end_to_end_p95_ms = NAN; h.last_measurement_age_ms = max(0.0, age) if math.isfinite(age) else NAN
        h.protocol_error_count = self.protocol_error_count; h.dropped_frame_count = self.sequence.dropped_count
        h.reconnect_count = self.stats['reconnect_count']; h.detector_backend = 'fast_v2'
        h.current_mode = self.mode; h.status_text = 'READY_PIXEL_ONLY' if h.algorithm_alive else 'WAITING_FOR_CAMERA'
        self.health_pub.publish(h)

    @staticmethod
    def _rate(values):
        return (len(values) - 1) / (values[-1] - values[0]) if len(values) > 1 and values[-1] > values[0] else 0.0

    @staticmethod
    def _pct(values, percentile):
        return sorted(values)[min(len(values) - 1, int(len(values) * percentile))] if values else NAN


def main(args=None):
    rclpy.init(args=args); node = VisionInterfaceNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
