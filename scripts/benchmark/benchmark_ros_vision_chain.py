#!/usr/bin/env python3
"""Collect read-only ROS V2 vision-chain acceptance metrics."""

import argparse
import json
import math
import os
import select
import subprocess
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


def device_snapshot(path='/dev/dtask_openmv'):
    present = os.path.exists(path)
    resolved = os.path.realpath(path) if present else ''
    props = {}
    if present:
        try:
            output = subprocess.check_output(
                ['udevadm', 'info', '--query=property', '--name', path],
                text=True, stderr=subprocess.DEVNULL)
            props = dict(line.split('=', 1) for line in output.splitlines() if '=' in line)
        except (OSError, subprocess.CalledProcessError):
            props = {}
    return {'present': present, 'path': resolved,
            'serial': props.get('ID_SERIAL_SHORT', ''), 'properties': props}


def rate_from_timestamps(times):
    """Steady-state rate, excluding startup time before the first sample."""
    if len(times) < 2 or times[-1] <= times[0]:
        return 0.0
    return (len(times) - 1) / (times[-1] - times[0])


class VisionChainBenchmark(Node):
    def __init__(self):
        super().__init__('benchmark_ros_vision_chain')
        from uav_interfaces.msg import TargetObservation, LandingError, VisionHealth
        self._message_types = (TargetObservation, LandingError, VisionHealth)
        self.started = time.monotonic(); self.raw = []; self.raw_by_sequence = {}; self.tracked = []
        self.tracked_sequences = set()
        self.landing = []; self.health = []; self.status = []
        self.sequences = SequenceTracker(); self.age = []; self.processing = []
        self.baseline_tracker = SequenceTracker(); self.post_tracker = SequenceTracker()
        self.stale = 0; self.disconnect = 0; self.reconnect = 0
        self.invalid_after_stale = False
        self.reconnect_seen = False; self.post_reconnect_sequences = set()
        self.initial_connected_count = 0; self.disconnected_count = 0
        self.stale_count = 0; self.recovered_count = 0
        self.baseline_sequences = set(); self.post_reconnect_raw = 0
        self.post_reconnect_tracked = 0; self.post_reconnect_landing = 0
        self.baseline_frame_times = []; self.post_raw_times = []
        self.baseline_tracked_times = []
        self.post_tracked_times = []; self.post_landing_times = []
        self.first_post_tracked_timestamp = None
        self.first_post_landing_timestamp = None
        self.baseline_stale_count = 0
        self.invalid_observation_after_disconnect = False
        self.ordered_disconnect_then_recovered = False
        self.device_path = '/dev/dtask_openmv'
        self.device_start = device_snapshot(self.device_path)
        self.device_absent_timestamp = None; self.device_present_timestamp = None
        self.device_absent_started_monotonic = None; self.device_present_monotonic = None
        self.device_absent_duration_sec = 0.0; self.device_after = None
        self.device_absence_qualified = False
        self.device_absence_qualified_monotonic = None
        self.device_poll_count_while_absent = 0
        self.device_current_present = self.device_start['present']
        self.disconnected_timestamp = None; self.disconnected_monotonic = None
        self.last_disconnected_monotonic = None
        self.disconnect_event_monotonic = None
        self.recovered_timestamp = None; self.recovered_monotonic = None
        self.disconnect_count_at_unplug_prompt = 0
        self.recovered_count_at_unplug_prompt = 0
        self.unplug_prompt_monotonic = None
        self.first_post_reconnect_frame_timestamp = None
        self.recovered_at = None
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
            self.post_tracker.accept(item['sequence'])
            self.post_reconnect_sequences.add(item['sequence'])
            self.post_reconnect_raw += 1
            self.post_raw_times.append(time.monotonic())
            if self.first_post_reconnect_frame_timestamp is None:
                self.first_post_reconnect_frame_timestamp = time.time()
        elif len(self.baseline_sequences) < 10000:
            self.baseline_tracker.accept(item['sequence'])
            self.baseline_sequences.add(item['sequence'])
            self.baseline_frame_times.append(time.monotonic())

    def tracked_cb(self, msg):
        self.tracked.append(msg)
        self.tracked_sequences.add(int(msg.frame_sequence))
        if self.reconnect_seen:
            self.post_reconnect_tracked += 1
            timestamp = time.monotonic(); self.post_tracked_times.append(timestamp)
            if self.first_post_tracked_timestamp is None:
                self.first_post_tracked_timestamp = timestamp
        else:
            self.baseline_tracked_times.append(time.monotonic())
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
            timestamp = time.monotonic(); self.post_landing_times.append(timestamp)
            if self.first_post_landing_timestamp is None:
                self.first_post_landing_timestamp = timestamp
    def health_cb(self, msg): self.health.append(msg)

    def status_cb(self, msg):
        self.status.append(msg.data)
        if msg.data == 'CONNECTED':
            self.initial_connected_count += 1
        if msg.data == 'STALE':
            self.stale += 1; self.stale_count += 1
            if not self.reconnect_seen:
                self.baseline_stale_count += 1
        if msg.data == 'DISCONNECTED':
            self.disconnect += 1; self.disconnected_count += 1
            self.last_disconnected_monotonic = time.monotonic()
            self.disconnect_event_monotonic = self.last_disconnected_monotonic
            if self.disconnected_timestamp is None:
                self.disconnected_timestamp = self.last_disconnected_monotonic
                self.disconnected_monotonic = self.disconnected_timestamp
            else:
                self.disconnected_monotonic = self.last_disconnected_monotonic
        if msg.data == 'RECOVERED':
            self.reconnect += 1; self.recovered_count += 1
            self.recovered_timestamp = time.monotonic()
            self.recovered_monotonic = self.recovered_timestamp
            if (self.disconnected_monotonic is not None and self.device_present_monotonic is not None and
                    self.recovered_monotonic >= self.device_present_monotonic and
                    self.unplug_prompt_monotonic is not None and
                    self.recovered_monotonic >= self.unplug_prompt_monotonic):
                self.ordered_disconnect_then_recovered = True
            self.reconnect_seen = True
            self.recovered_at = time.monotonic()
            self.post_tracker.reset()

    def poll_device(self):
        current = device_snapshot(self.device_path)
        self.device_current_present = current['present']
        now = time.time(); monotonic_now = time.monotonic()
        if not current['present']:
            if self.device_start['present']:
                if self.device_absent_started_monotonic is None:
                    self.device_absent_timestamp = now
                    self.device_absent_started_monotonic = monotonic_now
                self.device_poll_count_while_absent += 1
                self.device_absent_duration_sec = monotonic_now - self.device_absent_started_monotonic
                if self.device_absent_duration_sec >= 1.0:
                    self.device_absence_qualified = True
                    if self.device_absence_qualified_monotonic is None:
                        self.device_absence_qualified_monotonic = monotonic_now
        elif self.device_absent_started_monotonic is not None and self.device_present_timestamp is None:
            duration = monotonic_now - self.device_absent_started_monotonic
            self.device_absent_duration_sec = duration
            if duration >= 1.0 and self.device_absence_qualified:
                self.device_present_timestamp = now
                self.device_present_monotonic = monotonic_now
                self.device_after = current
            else:
                # A short disappearance is serial/udev jitter, not a test event.
                self.device_absent_timestamp = None
                self.device_absent_started_monotonic = None
                self.device_absent_duration_sec = 0.0
                self.device_absence_qualified = False
                self.device_absence_qualified_monotonic = None
                self.device_poll_count_while_absent = 0
        return current

    def physical_disconnect_ready(self):
        return (self.device_start['present'] and not self.device_current_present and
                self.device_absence_qualified and self.device_absent_timestamp is not None and
                self.device_absent_duration_sec >= 1.0 and getattr(self, 'disconnected_timestamp', None) is not None and
                self.disconnected_monotonic + 0.25 >= self.device_absent_started_monotonic and
                self.disconnected_monotonic + 0.25 >= self.unplug_prompt_monotonic and
                self.device_absent_started_monotonic + 0.25 >= self.unplug_prompt_monotonic and
                self.disconnected_count > self.disconnect_count_at_unplug_prompt and
                self.unplug_prompt_monotonic is not None)

    def report(self, seconds):
        unique = len(self.raw_by_sequence)
        valid = sum(int(item['valid']) for item in self.raw_by_sequence.values())
        confirmed = sum(int(msg.confirmed) for msg in self.tracked)
        landing_valid = sum(int(msg.valid) for msg in self.landing)
        health = self.health[-1] if self.health else None
        node_counts = self.node_instance_counts()
        baseline_hz = ((len(self.baseline_frame_times) - 1) /
                       (self.baseline_frame_times[-1] - self.baseline_frame_times[0])
                       if len(self.baseline_frame_times) > 1 and self.baseline_frame_times[-1] > self.baseline_frame_times[0] else 0.0)
        baseline_tracked_hz = ((len(self.baseline_tracked_times) - 1) /
                              (self.baseline_tracked_times[-1] - self.baseline_tracked_times[0])
                              if len(self.baseline_tracked_times) > 1 and self.baseline_tracked_times[-1] > self.baseline_tracked_times[0] else 0.0)
        post_end = time.monotonic()
        post_duration = max(0.001, post_end - self.recovered_at) if self.recovered_at else 0.001
        def window(times):
            return times[-1] - times[0] if len(times) >= 2 and times[-1] > times[0] else 0.0
        def stable_rate(times):
            span = window(times)
            return rate_from_timestamps(times) if span > 0.0 else 0.0
        return {
            'seconds': seconds, 'raw_target_v2_hz': len(self.raw) / max(seconds, 1e-9),
            'tracked_publish_hz': len(self.tracked) / max(seconds, 1e-9),
            'tracked_unique_frame_hz': len(self.tracked_sequences) / max(seconds, 1e-9),
            'valid_detection_hz': valid / max(seconds, 1e-9),
            'confirmed_detection_hz': confirmed / max(seconds, 1e-9),
            'landing_error_valid_hz': landing_valid / max(seconds, 1e-9),
            # Phase trackers intentionally exclude the OpenMV counter reset at reconnect.
            'duplicate_frame_sequence_count': (self.baseline_tracker.duplicate_count +
                                                self.post_tracker.duplicate_count),
            'sequence_gap_count': self.baseline_tracker.dropped_count + self.post_tracker.dropped_count,
            'out_of_order_sequence_count': (self.baseline_tracker.out_of_order_count +
                                            self.post_tracker.out_of_order_count),
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
            'baseline_raw_hz': baseline_hz,
            'baseline_tracked_hz': baseline_tracked_hz,
            'baseline_stale_count': self.baseline_stale_count,
            'post_reconnect_raw_frame_count': self.post_reconnect_raw,
            'post_reconnect_tracked_frame_count': self.post_reconnect_tracked,
            'post_reconnect_landing_frame_count': self.post_reconnect_landing,
            'post_reconnect_raw_hz': stable_rate(self.post_raw_times),
            'post_reconnect_tracked_hz': stable_rate(self.post_tracked_times),
            'post_reconnect_landing_hz': stable_rate(self.post_landing_times),
            'post_reconnect_effective_raw_hz_from_recovered': len(self.post_raw_times) / post_duration,
            'post_reconnect_effective_tracked_hz_from_recovered': len(self.post_tracked_times) / post_duration,
            'post_reconnect_effective_landing_hz_from_recovered': len(self.post_landing_times) / post_duration,
            'post_reconnect_raw_window_sec': window(self.post_raw_times),
            'post_reconnect_tracked_window_sec': window(self.post_tracked_times),
            'post_reconnect_landing_window_sec': window(self.post_landing_times),
            'post_reconnect_raw_first_monotonic': self.post_raw_times[0] if self.post_raw_times else None,
            'post_reconnect_raw_last_monotonic': self.post_raw_times[-1] if self.post_raw_times else None,
            'post_reconnect_tracked_first_monotonic': self.post_tracked_times[0] if self.post_tracked_times else None,
            'post_reconnect_tracked_last_monotonic': self.post_tracked_times[-1] if self.post_tracked_times else None,
            'post_reconnect_landing_first_monotonic': self.post_landing_times[0] if self.post_landing_times else None,
            'post_reconnect_landing_last_monotonic': self.post_landing_times[-1] if self.post_landing_times else None,
            'recovered_to_first_raw_frame_ms': ((self.post_raw_times[0] - self.recovered_at) * 1000.0
                                                if self.post_raw_times and self.recovered_at is not None else None),
            'recovered_to_first_tracked_frame_ms': ((self.post_tracked_times[0] - self.recovered_at) * 1000.0
                                                    if self.post_tracked_times and self.recovered_at is not None else None),
            'recovered_to_first_landing_frame_ms': ((self.post_landing_times[0] - self.recovered_at) * 1000.0
                                                    if self.post_landing_times and self.recovered_at is not None else None),
            'device_present_to_recovered_ms': ((self.recovered_monotonic - self.device_present_monotonic) * 1000.0
                                               if self.recovered_monotonic is not None and self.device_present_monotonic is not None else None),
            'invalid_observation_after_disconnect': self.invalid_observation_after_disconnect,
            'ordered_disconnect_then_recovered': self.ordered_disconnect_then_recovered,
            'launch_process_alive': self._nodes_alive(),
            'bridge_process_alive': 'h7_bridge_node' in self.get_node_names(),
            'interface_process_alive': 'vision_interface_node' in self.get_node_names(),
            'node_instance_counts': node_counts,
            'device_present_at_start': self.device_start['present'],
            'device_absent_observed': self.device_absent_timestamp is not None,
            'device_absence_qualified': self.device_absence_qualified,
            'device_absent_duration_sec': self.device_absent_duration_sec,
            'device_present_after_absent': self.device_present_timestamp is not None,
            'device_path_before': self.device_start['path'],
            'device_path_after': self.device_after['path'] if self.device_after else '',
            'usb_serial_before': self.device_start['serial'],
            'usb_serial_after': self.device_after['serial'] if self.device_after else '',
            'device_absent_timestamp': self.device_absent_timestamp,
            'device_absent_started_monotonic': self.device_absent_started_monotonic,
            'device_absence_qualified_monotonic': self.device_absence_qualified_monotonic,
            'device_poll_count_while_absent': self.device_poll_count_while_absent,
            'disconnected_timestamp': self.disconnected_timestamp,
            'disconnect_event_monotonic': self.disconnect_event_monotonic,
            'device_present_timestamp': self.device_present_timestamp,
            'recovered_timestamp': self.recovered_timestamp,
            'unplug_prompt_monotonic': self.unplug_prompt_monotonic,
            'disconnect_device_event_delta_ms': (
                (self.disconnect_event_monotonic - self.device_absent_started_monotonic) * 1000.0
                if self.disconnect_event_monotonic is not None and self.device_absent_started_monotonic is not None else None),
            'disconnect_count_at_unplug_prompt': self.disconnect_count_at_unplug_prompt,
            'recovered_count_at_unplug_prompt': self.recovered_count_at_unplug_prompt,
            'first_post_reconnect_frame_timestamp': self.first_post_reconnect_frame_timestamp,
            'baseline_duplicate_count': self.baseline_tracker.duplicate_count,
            'baseline_gap_count': self.baseline_tracker.dropped_count,
            'baseline_out_of_order_count': self.baseline_tracker.out_of_order_count,
            'post_duplicate_count': self.post_tracker.duplicate_count,
            'post_gap_count': self.post_tracker.dropped_count,
            'post_out_of_order_count': self.post_tracker.out_of_order_count,
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

    def node_instance_counts(self):
        names = self.get_node_names()
        return {'h7_bridge_node': names.count('h7_bridge_node'),
                'vision_interface_node': names.count('vision_interface_node'),
                'benchmark_ros_vision_chain': names.count('benchmark_ros_vision_chain')}


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
        if not report.get('device_present_at_start', False):
            failures.append('device_missing_at_start')
        if report.get('baseline_unique_frame_count', 0) < 60:
            failures.append('no_baseline_frames')
        if report.get('baseline_raw_hz', 0) < 30 or report.get('baseline_tracked_hz', 0) < 30:
            failures.append('baseline_frequency_below_30hz')
        if report.get('baseline_stale_count', 0) > 0:
            failures.append('baseline_stale')
        if report.get('baseline_gap_count', 0) > max(2, report.get('baseline_unique_frame_count', 0) * 0.10):
            failures.append('baseline_sequence_gaps')
        if not report.get('device_absent_observed', False) or report.get('device_absent_duration_sec', 0) < 1.0:
            failures.append('device_absence_not_confirmed')
        if report.get('disconnected_count', 0) <= 0 or not report.get('disconnected_timestamp'):
            failures.append('no_disconnected_after_device_absence')
        if report.get('recovered_count', 0) <= 0:
            failures.append('no_recovered')
        if not report.get('device_present_after_absent', False):
            failures.append('device_not_restored')
        if report.get('usb_serial_before') != report.get('usb_serial_after'):
            failures.append('usb_serial_mismatch')
        if not report.get('usb_serial_before') or not report.get('usb_serial_after'):
            failures.append('usb_serial_unavailable')
        if not report.get('ordered_disconnect_then_recovered', False):
            failures.append('bad_disconnect_recovered_order')
        if report.get('post_reconnect_unique_frame_count', 0) < 60:
            failures.append('insufficient_post_reconnect_frames')
        if min(report.get('post_reconnect_raw_frame_count', 0), report.get('post_reconnect_tracked_frame_count', 0), report.get('post_reconnect_landing_frame_count', 0)) < 60:
            failures.append('insufficient_post_reconnect_topic_frames')
        for name in ('raw', 'tracked', 'landing'):
            if report.get('post_reconnect_%s_hz' % name, 0) < 30:
                failures.append('post_reconnect_%s_frequency_below_30hz' % name)
            if report.get('post_reconnect_%s_window_sec' % name, 0) < 1.0:
                failures.append('post_reconnect_%s_window_too_short' % name)
        if report.get('post_gap_count', 0) > max(2, report.get('post_reconnect_unique_frame_count', 0) * 0.10):
            failures.append('post_sequence_gaps')
        if not report.get('invalid_observation_after_disconnect', False):
            failures.append('no_invalid_observation_after_disconnect')
        if not (report.get('launch_process_alive') and report.get('bridge_process_alive') and
                report.get('interface_process_alive')):
            failures.append('vision_nodes_not_alive')
        counts = report.get('node_instance_counts', {})
        if counts and (counts.get('h7_bridge_node') != 1 or counts.get('vision_interface_node') != 1):
            failures.append('duplicate_or_missing_vision_nodes')
    return failures


def classify_reconnect_failures(failures, phase):
    """Separate the first failed phase from checks that were never executable."""
    recovery_checks = {'no_recovered', 'device_not_restored', 'usb_serial_mismatch',
                       'usb_serial_unavailable', 'insufficient_post_reconnect_frames',
                       'insufficient_post_reconnect_topic_frames',
                       'post_reconnect_raw_frequency_below_30hz',
                       'post_reconnect_tracked_frequency_below_30hz',
                       'post_reconnect_landing_frequency_below_30hz',
                       'post_reconnect_raw_window_too_short',
                       'post_reconnect_tracked_window_too_short',
                       'post_reconnect_landing_window_too_short', 'post_sequence_gaps',
                       'bad_disconnect_recovered_order'}
    if phase == 'await_disconnect' and failures:
        primary = ['device_absence_qualification_state_machine_timeout']
        skipped = sorted(set(failures) & recovery_checks)
        secondary = [item for item in failures if item not in recovery_checks]
        return primary, skipped, secondary
    return list(failures[:1]), [], list(failures[1:])


def reconnect_run(node, seconds, baseline_timeout=30.0, disconnect_timeout=30.0,
                  recovery_timeout=45.0, post_timeout=30.0):
    """Event-driven reconnect acceptance; stdin is polled without stopping ROS spin."""
    started = time.monotonic(); phase = 'baseline'; phase_started = started
    prompted_disconnect = False; prompted_reconnect = False
    while time.monotonic() - started < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        node.poll_device()
        now = time.monotonic()
        baseline_duration = (node.baseline_frame_times[-1] - node.baseline_frame_times[0]
                             if len(node.baseline_frame_times) > 1 else 0.0)
        baseline_hz = ((len(node.baseline_frame_times) - 1) / baseline_duration
                       if baseline_duration > 0 else 0.0)
        if phase == 'baseline' and (len(node.baseline_sequences) >= 60 and
                                    baseline_duration >= 2.0 and baseline_hz >= 30.0 and
                                    node.tracked and node.landing and node.health and
                                    node.baseline_stale_count == 0):
            print('UNPLUG_OPENMV_NOW', file=sys.stderr, flush=True)
            node.disconnect_count_at_unplug_prompt = node.disconnected_count
            node.recovered_count_at_unplug_prompt = node.recovered_count
            node.unplug_prompt_monotonic = now
            phase = 'await_disconnect'; phase_started = now; prompted_disconnect = True
        if phase == 'await_disconnect' and node.physical_disconnect_ready():
            print('REPLUG_OPENMV_NOW', file=sys.stderr, flush=True)
            phase = 'await_reconnect'; phase_started = now; prompted_reconnect = True
        if (phase == 'await_reconnect' and node.device_present_timestamp is not None and
                node.recovered_count > node.recovered_count_at_unplug_prompt and
                node.ordered_disconnect_then_recovered):
            phase = 'post_reconnect'; phase_started = now
        if (phase == 'post_reconnect' and node.recovered_at is not None and
                now - node.recovered_at >= 2.0 and
                len(node.post_reconnect_sequences) >= 60 and
                node.post_reconnect_raw >= 60 and node.post_reconnect_tracked >= 60 and
                node.post_reconnect_landing >= 60 and
                all((len(stream) >= 2 and stream[-1] - stream[0] >= 1.0)
                    for stream in (node.post_raw_times, node.post_tracked_times, node.post_landing_times))):
            break
        limit = {'baseline': baseline_timeout, 'await_disconnect': disconnect_timeout,
                 'await_reconnect': recovery_timeout, 'post_reconnect': post_timeout}[phase]
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
        primary, skipped, secondary = classify_reconnect_failures(failures, phase)
        result['acceptance_status'] = 'FAIL' if failures else 'PASS'
        result['failure_reasons'] = failures
        result['primary_failure_reasons'] = primary
        result['skipped_checks'] = skipped
        result['secondary_failure_reasons'] = secondary
        node.destroy_node(); rclpy.shutdown()
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__': main()
