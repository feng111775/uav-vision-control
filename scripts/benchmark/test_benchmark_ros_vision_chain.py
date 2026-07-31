from benchmark_ros_vision_chain import (benchmark_qos_profile, classify_reconnect_failures,
                                        rate_from_timestamps, validate_report)
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rclpy
from rclpy.node import Node


def test_benchmark_qos_is_best_effort_volatile_keep_last():
    qos = benchmark_qos_profile()
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.history == HistoryPolicy.KEEP_LAST


def test_zero_messages_fail_acceptance():
    failures = validate_report({
        'raw_target_v2_hz': 0, 'tracked_publish_hz': 0,
        'tracked_unique_frame_hz': 0, 'health_message_count': 0,
        'landing_error_message_count': 0, 'protocol_error_count': 0,
        'px4_input_publisher_count': 0, 'processing_p50_ms': 0},)
    assert 'missing_raw_target_v2_hz' in failures
    assert 'missing_tracked_publish_hz' in failures
    assert 'missing_landing_error_message_count' in failures
    assert 'missing_health_message_count' in failures


def test_steady_rate_excludes_recovered_startup_delay():
    # RECOVERED at t=0, first frame at 1.5s, then 60 frames over ~1.28s (46Hz).
    times = [1.5 + i / 46.0 for i in range(60)]
    assert 45.0 < rate_from_timestamps(times) < 47.0
    effective = len(times) / times[-1]
    assert effective < rate_from_timestamps(times)


def test_steady_rate_requires_two_ordered_samples():
    assert rate_from_timestamps([]) == 0.0
    assert rate_from_timestamps([1.0]) == 0.0
    assert rate_from_timestamps([1.0, 1.0]) == 0.0
    assert rate_from_timestamps([2.0, 1.0]) == 0.0


def test_each_post_stream_has_its_own_frequency_failure():
    report = {
        'raw_target_v2_hz': 40, 'tracked_publish_hz': 40,
        'tracked_unique_frame_hz': 40, 'health_message_count': 2,
        'landing_error_message_count': 40, 'protocol_error_count': 0,
        'px4_input_publisher_count': 0, 'processing_p50_ms': 1,
        'post_reconnect_raw_hz': 46, 'post_reconnect_tracked_hz': 20,
        'post_reconnect_landing_hz': 46,
        'post_reconnect_raw_window_sec': 2, 'post_reconnect_tracked_window_sec': 2,
        'post_reconnect_landing_window_sec': 2}
    failures = validate_report(report, require_reconnect=True)
    assert 'post_reconnect_tracked_frequency_below_30hz' in failures


def test_reconnect_requires_disconnect_recovery_and_post_frame():
    report = {
        'raw_target_v2_hz': 20, 'tracked_publish_hz': 20,
        'tracked_unique_frame_hz': 20, 'health_message_count': 5,
        'landing_error_message_count': 20, 'protocol_error_count': 0,
        'px4_input_publisher_count': 0, 'processing_p50_ms': 1,
        'disconnect_transition_count': 1, 'stale_transition_count': 0,
        'reconnect_transition_count': 1, 'post_reconnect_unique_frame_count': 60,
        'baseline_unique_frame_count': 60, 'baseline_raw_hz': 30, 'baseline_tracked_hz': 30,
        'baseline_stale_count': 0, 'disconnected_count': 1,
        'stale_count': 0, 'recovered_count': 1,
        'device_present_at_start': True, 'device_absent_observed': True,
        'device_absent_duration_sec': 1.1, 'device_present_after_absent': True,
        'usb_serial_before': 'ABC', 'usb_serial_after': 'ABC',
        'disconnected_timestamp': 2, 'ordered_disconnect_then_recovered': True,
        'post_reconnect_raw_frame_count': 60, 'post_reconnect_tracked_frame_count': 60,
        'post_reconnect_landing_frame_count': 60, 'post_reconnect_raw_hz': 30,
        'post_reconnect_tracked_hz': 30, 'post_reconnect_landing_hz': 30,
        'post_reconnect_raw_window_sec': 2, 'post_reconnect_tracked_window_sec': 2,
        'post_reconnect_landing_window_sec': 2,
        'invalid_observation_after_disconnect': True,
        'launch_process_alive': True, 'bridge_process_alive': True,
        'interface_process_alive': True}
    assert validate_report(report, require_reconnect=True) == []


def test_reconnect_validation_rejects_missing_baseline_and_order():
    report = {
        'raw_target_v2_hz': 20, 'tracked_publish_hz': 20,
        'tracked_unique_frame_hz': 20, 'health_message_count': 2,
        'landing_error_message_count': 20, 'protocol_error_count': 0,
        'px4_input_publisher_count': 0, 'processing_p50_ms': 1,
        'baseline_unique_frame_count': 0, 'disconnected_count': 1,
        'stale_count': 0, 'recovered_count': 0,
        'ordered_disconnect_then_recovered': False,
        'post_reconnect_unique_frame_count': 0,
        'invalid_observation_after_disconnect': False,
        'launch_process_alive': True, 'bridge_process_alive': True,
        'interface_process_alive': True}
    failures = validate_report(report, require_reconnect=True)
    assert 'no_baseline_frames' in failures
    assert 'no_recovered' in failures
    assert 'bad_disconnect_recovered_order' in failures
    assert 'insufficient_post_reconnect_frames' in failures


def test_initial_connected_does_not_satisfy_reconnect():
    report = {
        'raw_target_v2_hz': 20, 'tracked_publish_hz': 20,
        'tracked_unique_frame_hz': 20, 'health_message_count': 2,
        'landing_error_message_count': 20, 'protocol_error_count': 0,
        'px4_input_publisher_count': 0, 'processing_p50_ms': 1,
        'baseline_unique_frame_count': 5, 'disconnected_count': 0,
        'stale_count': 0, 'recovered_count': 0,
        'initial_connected_count': 2, 'ordered_disconnect_then_recovered': False,
        'post_reconnect_unique_frame_count': 0,
        'invalid_observation_after_disconnect': False,
        'launch_process_alive': True, 'bridge_process_alive': True,
        'interface_process_alive': True}
    assert 'no_recovered' in validate_report(report, require_reconnect=True)


def test_reconnect_requires_real_device_events_and_stable_rates():
    report = {
        'raw_target_v2_hz': 40, 'tracked_publish_hz': 40,
        'tracked_unique_frame_hz': 40, 'health_message_count': 10,
        'landing_error_message_count': 40, 'protocol_error_count': 0,
        'px4_input_publisher_count': 0, 'processing_p50_ms': 1,
        'device_present_at_start': True, 'device_absent_observed': False,
        'baseline_unique_frame_count': 4, 'baseline_raw_hz': 2,
        'baseline_tracked_hz': 2, 'baseline_stale_count': 1,
        'disconnected_count': 1, 'recovered_count': 1,
        'device_present_after_absent': True, 'usb_serial_before': 'A',
        'usb_serial_after': 'A', 'post_reconnect_unique_frame_count': 1,
        'post_reconnect_raw_frame_count': 1, 'post_reconnect_tracked_frame_count': 1,
        'post_reconnect_landing_frame_count': 1, 'post_reconnect_raw_hz': 1,
        'post_reconnect_tracked_hz': 1, 'ordered_disconnect_then_recovered': True,
        'invalid_observation_after_disconnect': True,
        'launch_process_alive': True, 'bridge_process_alive': True,
        'interface_process_alive': True}
    failures = validate_report(report, require_reconnect=True)
    assert 'device_absence_not_confirmed' in failures
    assert 'baseline_frequency_below_30hz' in failures
    assert 'insufficient_post_reconnect_frames' in failures


def test_reconnect_rejects_status_only_without_device_serial():
    report = {
        'raw_target_v2_hz': 40, 'tracked_publish_hz': 40,
        'tracked_unique_frame_hz': 40, 'health_message_count': 10,
        'landing_error_message_count': 40, 'protocol_error_count': 0,
        'px4_input_publisher_count': 0, 'processing_p50_ms': 1,
        'device_present_at_start': True, 'device_absent_observed': True,
        'device_absent_duration_sec': 2, 'device_present_after_absent': True,
        'usb_serial_before': '', 'usb_serial_after': '',
        'baseline_unique_frame_count': 60, 'baseline_raw_hz': 30,
        'baseline_tracked_hz': 30, 'baseline_stale_count': 0,
        'disconnected_count': 1, 'disconnected_timestamp': 2,
        'recovered_count': 1, 'ordered_disconnect_then_recovered': True,
        'post_reconnect_unique_frame_count': 60,
        'post_reconnect_raw_frame_count': 60, 'post_reconnect_tracked_frame_count': 60,
        'post_reconnect_landing_frame_count': 60, 'post_reconnect_raw_hz': 30,
        'post_reconnect_tracked_hz': 30, 'invalid_observation_after_disconnect': True,
        'launch_process_alive': True, 'bridge_process_alive': True,
        'interface_process_alive': True}
    assert 'usb_serial_unavailable' in validate_report(report, require_reconnect=True)


def test_physical_disconnect_qualifies_while_device_is_still_absent():
    benchmark_module = __import__('benchmark_ros_vision_chain', fromlist=['VisionChainBenchmark'])
    node = benchmark_module.VisionChainBenchmark.__new__(benchmark_module.VisionChainBenchmark)
    node.device_start = {'present': True}
    node.device_current_present = False
    node.device_absence_qualified = True
    node.device_absent_timestamp = 100.0
    node.device_absent_duration_sec = 1.2
    node.device_absent_started_monotonic = 10.0
    node.disconnected_monotonic = 10.1
    node.disconnected_timestamp = 10.1
    node.disconnect_count = 1
    node.disconnected_count = 1
    node.disconnect_count_at_unplug_prompt = 0
    node.unplug_prompt_monotonic = 9.0
    assert node.physical_disconnect_ready()


def test_physical_disconnect_rejects_status_without_device_absence():
    benchmark_module = __import__('benchmark_ros_vision_chain', fromlist=['VisionChainBenchmark'])
    node = benchmark_module.VisionChainBenchmark.__new__(benchmark_module.VisionChainBenchmark)
    node.device_start = {'present': True}
    node.device_current_present = True
    node.device_absence_qualified = False
    node.device_absent_timestamp = None
    node.device_absent_duration_sec = 0.0
    node.device_absent_started_monotonic = None
    node.disconnected_monotonic = 5.0
    node.disconnect_count = 1
    node.disconnect_count_at_unplug_prompt = 0
    node.unplug_prompt_monotonic = 1.0
    assert not node.physical_disconnect_ready()


def test_reconnect_failure_report_is_phase_aware():
    primary, skipped, secondary = classify_reconnect_failures(
        ['device_absence_not_confirmed', 'no_recovered', 'usb_serial_mismatch'],
        'await_disconnect')
    assert primary == ['device_absence_qualification_state_machine_timeout']
    assert skipped == ['no_recovered', 'usb_serial_mismatch']
    assert secondary == ['device_absence_not_confirmed']


def _poll_fixture(module):
    node = module.VisionChainBenchmark.__new__(module.VisionChainBenchmark)
    node.device_path = '/dev/test-openmv'
    node.device_start = {'present': True}
    node.device_current_present = True
    node.device_absent_timestamp = None
    node.device_absent_started_monotonic = None
    node.device_present_timestamp = None
    node.device_present_monotonic = None
    node.device_absent_duration_sec = 0.0
    node.device_after = None
    node.device_absence_qualified = False
    node.device_absence_qualified_monotonic = None
    node.device_poll_count_while_absent = 0
    return node


def test_poll_device_accumulates_continuous_absence(monkeypatch):
    import benchmark_ros_vision_chain as module
    node = _poll_fixture(module)
    wall = iter([100.0, 100.5, 101.0, 102.0])
    mono = iter([10.0, 10.5, 11.0, 12.0])
    monkeypatch.setattr(module.time, 'time', lambda: next(wall))
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(mono))
    monkeypatch.setattr(module, 'device_snapshot', lambda _: {'present': False, 'path': '', 'serial': ''})
    node.poll_device()
    assert node.device_absent_duration_sec == 0.0 and not node.device_absence_qualified
    node.poll_device()
    assert node.device_absent_duration_sec == 0.5 and not node.device_absence_qualified
    node.poll_device()
    assert node.device_absence_qualified and node.device_absent_duration_sec == 1.0
    node.poll_device()
    assert node.device_absent_duration_sec == 2.0


def test_poll_device_short_disappearance_resets_as_jitter(monkeypatch):
    import benchmark_ros_vision_chain as module
    node = _poll_fixture(module)
    wall = iter([10.0, 10.4])
    mono = iter([10.0, 10.4])
    monkeypatch.setattr(module.time, 'time', lambda: next(wall))
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(mono))
    states = iter([False, True])
    monkeypatch.setattr(module, 'device_snapshot', lambda _: {'present': next(states), 'path': '/dev/test', 'serial': 'S'})
    node.poll_device(); node.poll_device()
    assert node.device_absent_started_monotonic is None
    assert not node.device_absence_qualified and node.device_absent_duration_sec == 0.0


def test_best_effort_publishers_reach_benchmark_topics():
    import pytest
    if __import__('importlib').util.find_spec('uav_interfaces') is None:
        pytest.skip('uav_interfaces overlay is not sourced for standalone Python tests')
    rclpy.init(args=None)
    benchmark = None; publisher_node = None
    try:
        from std_msgs.msg import String
        from uav_interfaces.msg import TargetObservation, LandingError, VisionHealth
        benchmark = __import__('benchmark_ros_vision_chain', fromlist=['VisionChainBenchmark']).VisionChainBenchmark()
        publisher_node = Node('qos_test_publishers')
        qos = benchmark_qos_profile()
        raw_pub = publisher_node.create_publisher(String, '/vision/internal/h7/raw', qos)
        tracked_pub = publisher_node.create_publisher(TargetObservation, '/vision/target/tracked', qos)
        landing_pub = publisher_node.create_publisher(LandingError, '/vision/landing_error', qos)
        health_pub = publisher_node.create_publisher(VisionHealth, '/vision/health', qos)
        for _ in range(10):
            raw = String(); raw.data = 'D_TARGET_V2,1,100,1000,SEARCH,1,160,120,50,30,0.0,80'
            tracked = TargetObservation(); tracked.frame_sequence = 1; tracked.measurement_valid = True
            landing = LandingError(); landing.frame_sequence = 1
            health = VisionHealth(); health.new_frame_fps = 20.0
            raw_pub.publish(raw); tracked_pub.publish(tracked)
            landing_pub.publish(landing); health_pub.publish(health)
            rclpy.spin_once(benchmark, timeout_sec=0.02)
            rclpy.spin_once(publisher_node, timeout_sec=0.02)
        assert benchmark.raw and benchmark.tracked and benchmark.landing and benchmark.health
    finally:
        if benchmark is not None: benchmark.destroy_node()
        if publisher_node is not None: publisher_node.destroy_node()
        rclpy.shutdown()
