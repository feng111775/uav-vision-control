from benchmark_openmv_serial import (
    DETECT_STATS_FIELD_COUNT,
    DETECT_STATS_INDEX,
    analyze_lines,
)


def make_detect_stats_line(**overrides):
    fields = [''] * DETECT_STATS_FIELD_COUNT
    values = {
        'prefix': 'D_DETECT_STATS',
        'frame_sequence': '3',
        'mode': 'SEARCH',
        'blob_count': '4',
        'region_count': '2',
        'strong_verify_due_count': '1',
        'strong_verify_attempt_count': '2',
        'strong_verify_success_count': '1',
        'strong_verify_failure_count': '1',
        'circle_count': '6',
        'circle_pair_count': '1',
        'best_cross_score': '0.710',
        'best_confidence': '62.000',
        'best_ratio': '0.600',
        'cross_line_count': '4',
        'cross_candidate_count': '3',
        'last_verified_age': '1',
        'current_measurement_count': '2',
        'current_valid_frame_count': '1',
        'verification_grace_frame_count': '1',
        'candidate_switch_count': '5',
        'track_jump_px': '4.000',
        'track_jump_diameter_ratio': '0.120',
        'selected_blob_diameter_min': '58.0',
        'selected_blob_diameter_max': '60.0',
        'selected_blob_center_range': '120:90:122:93',
        'expected_outer_diameter': '59.0',
        'current_blob_diameter': '60.0',
        'hough_radius_min': '25.0',
        'hough_radius_max': '35.0',
        'edge_clipped_count': '0',
        'full_search_reset_count': '1',
        'acquire_timeout_count': '0',
        'verified_track_reset_count': '0',
        'reject_payload': 'NO_BLOB:1|VALID:1',
    }
    values.update(overrides)
    for name, index in DETECT_STATS_INDEX.items():
        fields[index] = values[name]
    return ','.join(fields) + '\n'


def test_detect_stats_line_schema_matches_device_layout():
    line = make_detect_stats_line()
    assert len(line.strip().split(',')) == 35
    report = analyze_lines([
        'D_CONFIG,MIN_CONFIDENCE=55,CONFIG_REPEAT_PERIOD_MS=10000\n',
        line,
        'D_TARGET_V2,3,20,2000,SEARCH,1,1,2,30,18,0.1000,80\n',
    ], 1)
    assert report['detect_stats_count'] == 1
    assert report['malformed_count'] == 0
    assert report['candidate_stage_counts']['strong_verify_attempt_count'] == 2
    assert report['candidate_stage_counts']['candidate_switch_count'] == 5
    assert report['candidate_stage_counts']['verification_grace_frame_count'] == 1
    assert report['candidate_stage_counts']['edge_clipped_count'] == 0
    assert report['candidate_stage_counts']['full_search_reset_count'] == 1
    assert report['candidate_stage_counts']['acquire_timeout_count'] == 0
    assert report['candidate_stage_counts']['verified_track_reset_count'] == 0
    assert report['reject_reason_counts']['NO_BLOB'] == 1
    assert report['best_confidence_range'] == [62.0, 62.0]
    assert report['track_jump_px_range'] == [4.0, 4.0]
    assert report['track_jump_diameter_ratio_range'] == [0.12, 0.12]
    assert report['selected_blob_diameter_min_range'] == [58.0, 58.0]
    assert report['selected_blob_diameter_max_range'] == [60.0, 60.0]
    assert report['selected_blob_center_range'] == [120, 90, 122, 93]
    assert report['expected_outer_diameter_range'] == [59.0, 59.0]
    assert report['current_blob_diameter_range'] == [60.0, 60.0]
    assert report['hough_radius_min_range'] == [25.0, 25.0]
    assert report['hough_radius_max_range'] == [35.0, 35.0]
    assert report['runtime_config']['MIN_CONFIDENCE'] == '55'
    assert 0.0 <= report['strong_verify_success_rate'] <= 1.0
    assert report['strong_verify_success_rate'] == 0.5


def test_report_marks_missing_protocol():
    report = analyze_lines(['D_VISION,status=LOST,fps=3.5\n'], 10)
    assert report['error'] == 'PROTOCOL_V2_MISSING'


def test_detect_stats_missing_fields_is_malformed():
    report = analyze_lines([
        make_detect_stats_line().replace(',NO_BLOB:1|VALID:1\n', '\n'),
        'D_TARGET_V2,1,10,1000,SEARCH,0,0,0,0,0,0.0000,0\n',
    ], 1)
    assert report['detect_stats_count'] == 0
    assert report['malformed_reasons']['detect_stats_missing_fields'] == 1


def test_detect_stats_extra_fields_is_malformed():
    report = analyze_lines([
        make_detect_stats_line().rstrip('\n') + ',EXTRA\n',
        'D_TARGET_V2,1,10,1000,SEARCH,0,0,0,0,0,0.0000,0\n',
    ], 1)
    assert report['detect_stats_count'] == 0
    assert report['malformed_reasons']['detect_stats_extra_fields'] == 1


def test_detect_stats_invalid_numeric_is_malformed():
    report = analyze_lines([
        make_detect_stats_line(track_jump_px='bad'),
        'D_TARGET_V2,1,10,1000,SEARCH,0,0,0,0,0,0.0000,0\n',
    ], 1)
    assert report['detect_stats_count'] == 0
    assert any(reason.startswith('detect_stats_invalid_') for reason in report['malformed_reasons'])


def test_runtime_config_can_arrive_without_boot_line():
    report = analyze_lines([
        'D_STATUS_V2,20,SEARCH,1,1,1,0,23.1\n',
        'D_CONFIG,MIN_CONFIDENCE=55,MAX_STRONG_VERIFY_AGE=12\n',
        'D_TARGET_V2,20,30,1100,SEARCH,0,0,0,0,0,0.0000,0\n',
    ], 1)
    assert report['runtime_config']['MIN_CONFIDENCE'] == '55'


def test_edge_clipped_target_rejected():
    report = analyze_lines([
        make_detect_stats_line(edge_clipped_count='3', reject_payload='EDGE_CLIPPED:3'),
        'D_TARGET_V2,1,10,1000,SEARCH,0,0,0,0,0,0.0000,0\n',
    ], 1)
    assert report['candidate_stage_counts']['edge_clipped_count'] == 3
    assert report['reject_reason_counts']['EDGE_CLIPPED'] == 3


def test_processing_p99_and_max_reported():
    report = analyze_lines([
        'D_TARGET_V2,1,10,1000,SEARCH,1,25,2,30,18,0.1000,80\n',
        'D_TARGET_V2,2,15,1500,SEARCH,1,30,3,32,19,0.1100,82\n',
        'D_TARGET_V2,3,20,2000,SEARCH,1,35,4,35,20,0.1200,85\n',
        'D_TARGET_V2,4,25,250000,SEARCH,1,40,5,38,22,0.1300,90\n',
    ], 4)
    assert 'processing_p99_ms' in report
    assert 'processing_max_ms' in report
    assert report['processing_max_ms'] == 250.0


def test_strong_verify_success_rate_bounded():
    report = analyze_lines([
        make_detect_stats_line(strong_verify_attempt_count='5', strong_verify_success_count='3'),
        'D_TARGET_V2,1,10,1000,SEARCH,0,0,0,0,0,0.0000,0\n',
    ], 1)
    assert 0.0 <= report['strong_verify_success_rate'] <= 1.0
    assert report['strong_verify_success_rate'] == 0.6
