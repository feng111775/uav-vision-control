from benchmark_openmv_serial import analyze_lines

def test_report_requires_v2_and_counts_sequences():
    lines = [
        'D_BOOT_V2,transport=stdout,vcp_id=0,backend=fast_v2\n',
        'D_TARGET_V2,1,10,1000,SEARCH,0,0,0,0,0,0.0000,0\n',
        'D_TARGET_V2,3,20,2000,SEARCH,1,1,2,30,18,0.1000,80\n',
        'D_PROTOCOL_ERROR,count=1,type=IOError,message=x\n',
    ]
    report = analyze_lines(lines, 1)
    assert report['boot_v2_seen'] and report['unique_sequence_count'] == 2
    assert report['sequence_gap_count'] == 1 and report['protocol_error_count'] == 1
    assert report['valid_detection_hz'] == 1

def test_report_marks_missing_protocol():
    report = analyze_lines(['D_VISION,status=LOST,fps=3.5\n'], 10)
    assert report['error'] == 'PROTOCOL_V2_MISSING'
