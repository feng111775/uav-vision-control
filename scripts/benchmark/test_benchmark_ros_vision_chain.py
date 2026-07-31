from benchmark_ros_vision_chain import benchmark_qos_profile, validate_report
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


def test_reconnect_requires_disconnect_recovery_and_post_frame():
    report = {
        'raw_target_v2_hz': 20, 'tracked_publish_hz': 20,
        'tracked_unique_frame_hz': 20, 'health_message_count': 5,
        'landing_error_message_count': 20, 'protocol_error_count': 0,
        'px4_input_publisher_count': 0, 'processing_p50_ms': 1,
        'disconnect_transition_count': 1, 'stale_transition_count': 0,
        'reconnect_transition_count': 2, 'post_reconnect_unique_frame_count': 3,
        'baseline_unique_frame_count': 5, 'disconnected_count': 1,
        'stale_count': 0, 'recovered_count': 1,
        'ordered_disconnect_then_recovered': True,
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
    assert 'no_post_reconnect_frame' in failures


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
