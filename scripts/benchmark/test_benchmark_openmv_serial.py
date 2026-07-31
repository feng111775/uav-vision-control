from benchmark_openmv_serial import analyze_lines


def test_report_requires_v2_and_counts_sequences_and_diagnostics():
    lines = [
        'D_BOOT_V2,transport=stdout,vcp_id=0,backend=fast_v2\n',
        'D_CONFIG,MIN_CONFIDENCE=55,CIRCLE_THRESHOLD=1400,LINE_THRESHOLD=550\n',
        'D_DETECT_STATS,3,SEARCH,4,2,1,2,6,1,4,0.710,62.000,0.600,3,1,1,NO_BLOB:1|VALID:1\n',
        'D_TARGET_V2,1,10,1000,SEARCH,0,0,0,0,0,0.0000,0\n',
        'D_TARGET_V2,3,20,2000,SEARCH,1,1,2,30,18,0.1000,80\n',
        'D_PROTOCOL_ERROR,count=1,type=IOError,message=x\n',
    ]
    report = analyze_lines(lines, 1)
    assert report['boot_v2_seen'] and report['unique_sequence_count'] == 2
    assert report['sequence_gap_count'] == 1 and report['protocol_error_count'] == 1
    assert report['valid_detection_hz'] == 1
    assert report['candidate_stage_counts']['verify_attempt_count'] == 2
    assert report['reject_reason_counts']['NO_BLOB'] == 1
    assert report['strong_verify_success_rate'] == 0.5
    assert report['runtime_config']['MIN_CONFIDENCE'] == '55'
    assert report['best_confidence_range'] == [62.0, 62.0]
    assert report['best_circle_ratio_range'] == [0.6, 0.6]
    assert report['best_cross_score_range'] == [0.71, 0.71]


def test_report_marks_missing_protocol():
    report = analyze_lines(['D_VISION,status=LOST,fps=3.5\n'], 10)
    assert report['error'] == 'PROTOCOL_V2_MISSING'


def test_reject_reason_aggregation_handles_multiple_reasons():
    report = analyze_lines([
        'D_BOOT_V2,transport=stdout,vcp_id=0,backend=fast_v2\n',
        'D_DETECT_STATS,8,SEARCH,1,0,1,1,0,0,0,0.000,0.000,0.000,0,13,0,NO_BLOB:2|VERIFY_EXPIRED:1|CROSS_SCORE_LOW:3\n',
        'D_TARGET_V2,8,20,1200,SEARCH,0,0,0,0,0,0.0000,0\n',
    ], 1)
    assert report['reject_reason_counts'] == {
        'NO_BLOB': 2,
        'VERIFY_EXPIRED': 1,
        'CROSS_SCORE_LOW': 3,
    }
