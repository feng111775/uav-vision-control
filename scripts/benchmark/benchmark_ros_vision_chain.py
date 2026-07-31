#!/usr/bin/env python3
"""Collect read-only ROS V2 vision-chain acceptance metrics."""

import argparse
import json
import math
import select
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

from uav_vision.vision_protocol import parse_target, SequenceTracker


def benchmark_qos_profile():
    return QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST)


class VisionChainBenchmark(Node):
    def __init__(self):
        super().__init__('benchmark_ros_vision_chain')
        from uav_interfaces.msg import TargetObservation, LandingError, VisionHealth
        self._message_types = (TargetObservation, LandingError, VisionHealth)
        self.started = time.monotonic(); self.raw = []; self.raw_by_sequence = {}; self.tracked = []
        self.tracked_sequences = set()
        self.landing = []; self.health = []; self.status = []
        self.sequences = SequenceTracker(); self.age = []; self.processing = []
        self.stale = 0; self.disconnect = 0; self.reconnect = 0
        self.invalid_after_stale = False
        self.reconnect_seen = False; self.post_reconnect_sequences = set()
        self.initial_connected_count = 0; self.disconnected_count = 0
        self.stale_count = 0; self.recovered_count = 0
        self.baseline_sequences = set(); self.post_reconnect_raw = 0
        self.post_reconnect_tracked = 0; self.post_reconnect_landing = 0
        self.invalid_observation_after_disconnect = False
        self.ordered_disconnect_then_recovered = False
        qos = benchmark_qos_profile()
        self.create_subscription(String, '/vision/internal/h7/raw', self.raw_cb, qos)
        self.create_subscription(self._message_types[0], '/vision/target/tracked', self.tracked_cb, qos)
        self.create_subscription(self._message_types[1], '/vision/landing_error', self.landing_cb, qos)
        self.create_subscription(self._message_types[2], '/vision/health', self.health_cb, qos)
        self.create_subscription(String, '/vision/h7/status', self.status_cb, qos)

    def raw_cb(self, msg):
        try: item = parse_target(msg.data)
        except ValueError: return
        if item.get('version') != 2: return
        self.raw.append(item)
        self.raw_by_sequence[item['sequence']] = item
        self.processing.append(item['processing_us'] / 1000.0)
        self.sequences.accept(item['sequence'])
        if self.reconnect_seen:
            self.post_reconnect_sequences.add(item['sequence'])
            self.post_reconnect_raw += 1
        elif len(self.baseline_sequences) < 10000:
            self.baseline_sequences.add(item['sequence'])

    def tracked_cb(self, msg):
        self.tracked.append(msg)
        self.tracked_sequences.add(int(msg.frame_sequence))
        if self.reconnect_seen:
            self.post_reconnect_tracked += 1
        if self.stale and not msg.measurement_valid:
            self.invalid_after_stale = True
        if (self.disconnected_count or self.stale_count) and not msg.measurement_valid:
            self.invalid_observation_after_disconnect = True
        if math.isfinite(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9):
            self.age.append(max(0.0, (self.get_clock().now().nanoseconds -
                                      (msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec)) / 1e6))

    def landing_cb(self, msg):
        self.landing.append(msg)
        if self.reconnect_seen:
            self.post_reconnect_landing += 1
    def health_cb(self, msg): self.health.append(msg)

    def status_cb(self, msg):
        self.status.append(msg.data)
        if msg.data == 'CONNECTED':
            self.initial_connected_count += 1
        if msg.data == 'STALE':
            self.stale += 1; self.stale_count += 1
        if msg.data == 'DISCONNECTED':
            self.disconnect += 1; self.disconnected_count += 1
        if msg.data == 'RECOVERED':
            self.reconnect += 1; self.recovered_count += 1
            if self.disconnected_count or self.stale_count:
                self.ordered_disconnect_then_recovered = True
            self.reconnect_seen = True

    def report(self, seconds):
        unique = len(self.raw_by_sequence)
        valid = sum(int(item['valid']) for item in self.raw_by_sequence.values())
        confirmed = sum(int(msg.confirmed) for msg in self.tracked)
        landing_valid = sum(int(msg.valid) for msg in self.landing)
        health = self.health[-1] if self.health else None
        return {
            'seconds': seconds, 'raw_target_v2_hz': len(self.raw) / max(seconds, 1e-9),
            'tracked_publish_hz': len(self.tracked) / max(seconds, 1e-9),
            'tracked_unique_frame_hz': len(self.tracked_sequences) / max(seconds, 1e-9),
            'valid_detection_hz': valid / max(seconds, 1e-9),
            'confirmed_detection_hz': confirmed / max(seconds, 1e-9),
            'landing_error_valid_hz': landing_valid / max(seconds, 1e-9),
            'duplicate_frame_sequence_count': self.sequences.duplicate_count,
            'sequence_gap_count': self.sequences.dropped_count,
            'out_of_order_sequence_count': self.sequences.out_of_order_count,
            'protocol_error_count': int(getattr(health, 'protocol_error_count', 0) if health else 0),
            'capture_stamp_valid_ratio': (sum(int(m.capture_stamp_valid) for m in self.tracked) /
                                          len(self.tracked) if self.tracked else 0.0),
            'measurement_age_p50_ms': self._pct(self.age, .50),
            'measurement_age_p95_ms': self._pct(self.age, .95),
            'measurement_age_p99_ms': self._pct(self.age, .99),
            'measurement_age_max_ms': max(self.age) if self.age else 0.0,
            'processing_p50_ms': self._pct(self.processing, .50),
            'processing_p95_ms': self._pct(self.processing, .95),
            'processing_p99_ms': self._pct(self.processing, .99),
            'stale_transition_count': self.stale,
            'disconnect_transition_count': self.disconnect,
            'reconnect_transition_count': self.reconnect,
            'invalid_after_stale': self.invalid_after_stale,
            'px4_input_publisher_count': self.px4_input_publishers(),
            'post_reconnect_unique_frame_count': len(self.post_reconnect_sequences),
            'initial_connected_count': self.initial_connected_count,
            'disconnected_count': self.disconnected_count,
            'stale_count': self.stale_count,
            'recovered_count': self.recovered_count,
            'baseline_unique_frame_count': len(self.baseline_sequences),
            'post_reconnect_raw_frame_count': self.post_reconnect_raw,
            'post_reconnect_tracked_frame_count': self.post_reconnect_tracked,
            'post_reconnect_landing_frame_count': self.post_reconnect_landing,
            'invalid_observation_after_disconnect': self.invalid_observation_after_disconnect,
            'ordered_disconnect_then_recovered': self.ordered_disconnect_then_recovered,
            'launch_process_alive': self._nodes_alive(),
            'bridge_process_alive': 'h7_bridge_node' in self.get_node_names(),
            'interface_process_alive': 'vision_interface_node' in self.get_node_names(),
        }

    @staticmethod
    def _pct(values, p):
        return sorted(values)[min(len(values) - 1, int(len(values) * p))] if values else 0.0

    def px4_input_publishers(self):
        count = 0
        for topic, _ in self.get_topic_names_and_types():
            if topic.startswith('/fmu/in/'):
                count += len(self.get_publishers_info_by_topic(topic))
        return count

    def _nodes_alive(self):
        names = self.get_node_names()
        return 'h7_bridge_node' in names and 'vision_interface_node' in names


def validate_report(report, require_reconnect=False):
    failures = []
    for key in ('raw_target_v2_hz', 'tracked_publish_hz', 'tracked_unique_frame_hz',
                'health_message_count', 'landing_error_message_count'):
        if report.get(key, 0) <= 0:
            failures.append('missing_%s' % key)
    if report.get('protocol_error_count', 0) != 0:
        failures.append('protocol_error_count')
    if report.get('px4_input_publisher_count', 0) != 0:
        failures.append('px4_input_publishers')
    if report.get('processing_p50_ms') is None:
        failures.append('processing_metric_unavailable')
    if require_reconnect:
        if report.get('baseline_unique_frame_count', 0) <= 0:
            failures.append('no_baseline_frames')
        if report.get('disconnected_count', 0) + report.get('stale_count', 0) <= 0:
            failures.append('no_disconnect_or_stale')
        if report.get('recovered_count', 0) <= 0:
            failures.append('no_recovered')
        if not report.get('ordered_disconnect_then_recovered', False):
            failures.append('bad_disconnect_recovered_order')
        if report.get('post_reconnect_unique_frame_count', 0) <= 0:
            failures.append('no_post_reconnect_frame')
        if not report.get('invalid_observation_after_disconnect', False):
            failures.append('no_invalid_observation_after_disconnect')
        if not (report.get('launch_process_alive') and report.get('bridge_process_alive') and
                report.get('interface_process_alive')):
            failures.append('vision_nodes_not_alive')
    return failures


def reconnect_run(node, seconds, baseline_timeout=30.0, disconnect_timeout=30.0,
                  recovery_timeout=45.0, post_timeout=30.0):
    """Event-driven reconnect acceptance; stdin is polled without stopping ROS spin."""
    started = time.monotonic(); phase = 'baseline'; phase_started = started
    prompted_disconnect = False; prompted_reconnect = False
    while time.monotonic() - started < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        if phase == 'baseline' and node.baseline_sequences and node.tracked and node.landing and node.health:
            print('BASELINE_READY: unplug OpenMV, then press Enter', file=sys.stderr, flush=True)
            phase = 'await_disconnect'; phase_started = now; prompted_disconnect = True
        if phase == 'await_disconnect' and sys.stdin.isatty():
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                sys.stdin.readline(); phase = 'disconnect'; phase_started = now
                print('DISCONNECT_REQUESTED: remove OpenMV', file=sys.stderr, flush=True)
        if phase in ('disconnect', 'await_disconnect') and (node.disconnected_count or node.stale_count):
            print('DISCONNECT_OBSERVED: insert OpenMV again, then press Enter', file=sys.stderr, flush=True)
            phase = 'await_reconnect'; phase_started = now; prompted_reconnect = True
        if phase == 'await_reconnect' and sys.stdin.isatty():
            ready, _, _ = select.select([sys.stdin], [], [], 0)
            if ready:
                sys.stdin.readline(); phase = 'reconnect'; phase_started = now
                print('RECONNECT_REQUESTED: waiting for RECOVERED and new frames', file=sys.stderr, flush=True)
        if phase in ('reconnect', 'await_reconnect') and node.recovered_count and node.reconnect_seen:
            phase = 'post_reconnect'; phase_started = now
        if phase == 'post_reconnect' and node.post_reconnect_sequences and node.post_reconnect_tracked and node.post_reconnect_landing:
            break
        limit = {'baseline': baseline_timeout, 'await_disconnect': baseline_timeout,
                 'disconnect': disconnect_timeout, 'await_reconnect': disconnect_timeout,
                 'reconnect': recovery_timeout, 'post_reconnect': post_timeout}[phase]
        if now - phase_started > limit:
            break
    return phase


def main():
    parser = argparse.ArgumentParser(); parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--require-reconnect', action='store_true'); args = parser.parse_args()
    rclpy.init(); node = VisionChainBenchmark(); start = time.monotonic(); phase = None
    try:
        if args.require_reconnect:
            phase = reconnect_run(node, args.seconds)
        else:
            while time.monotonic() - start < args.seconds:
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        result = node.report(time.monotonic() - start)
        result['tracked_message_count'] = len(node.tracked)
        result['landing_error_message_count'] = len(node.landing)
        result['health_message_count'] = len(node.health)
        result['reconnect_phase'] = phase
        failures = validate_report(result, args.require_reconnect)
        result['acceptance_status'] = 'FAIL' if failures else 'PASS'
        result['failure_reasons'] = failures
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__': main()
