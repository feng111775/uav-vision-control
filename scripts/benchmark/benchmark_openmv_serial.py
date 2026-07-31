#!/usr/bin/env python3
"""Benchmark and validate the OpenMV V2 serial protocol."""
import argparse
import json
import re
import sys
import time

try:
    import serial
except ImportError:
    serial = None

DETECT_STATS_PREFIX = 'D_DETECT_STATS,'
CONFIG_PREFIX = 'D_CONFIG,'

DETECT_STATS_FIELDS = [
    'prefix', 'frame_sequence', 'mode', 'blob_count', 'region_count',
    'strong_verify_due_count', 'strong_verify_attempt_count',
    'strong_verify_success_count', 'strong_verify_failure_count',
    'circle_count', 'circle_pair_count', 'best_cross_score',
    'best_confidence', 'best_ratio', 'cross_line_count',
    'cross_candidate_count', 'last_verified_age',
    'current_measurement_count', 'current_valid_frame_count',
    'verification_grace_frame_count', 'candidate_switch_count',
    'track_jump_px', 'track_jump_diameter_ratio',
    'selected_blob_diameter_min', 'selected_blob_diameter_max',
    'selected_blob_center_range',
    'expected_outer_diameter', 'current_blob_diameter',
    'hough_radius_min', 'hough_radius_max',
    'edge_clipped_count', 'verify_roi_clamped_count', 'full_search_reset_count',
    'acquire_timeout_count', 'verified_track_reset_count',
    'reject_payload',
]
DETECT_STATS_INDEX = {name: index for index, name in enumerate(DETECT_STATS_FIELDS)}
DETECT_STATS_FIELD_COUNT = len(DETECT_STATS_FIELDS)

INTEGER_FIELDS = (
    'frame_sequence', 'blob_count', 'region_count', 'strong_verify_due_count',
    'strong_verify_attempt_count', 'strong_verify_success_count',
    'strong_verify_failure_count', 'circle_count', 'circle_pair_count',
    'cross_line_count', 'cross_candidate_count', 'last_verified_age',
    'current_measurement_count', 'current_valid_frame_count',
    'verification_grace_frame_count', 'candidate_switch_count',
    'edge_clipped_count', 'full_search_reset_count',
    'verify_roi_clamped_count',
    'acquire_timeout_count', 'verified_track_reset_count',
)
FLOAT_FIELDS = (
    'best_cross_score', 'best_confidence', 'best_ratio', 'track_jump_px',
    'track_jump_diameter_ratio', 'selected_blob_diameter_min',
    'selected_blob_diameter_max', 'expected_outer_diameter',
    'current_blob_diameter', 'hough_radius_min', 'hough_radius_max',
)


def _parse_reject_counts(payload):
    counts = {}
    for item in str(payload or '').split('|'):
        if not item or ':' not in item:
            continue
        reason, raw = item.split(':', 1)
        try:
            counts[reason] = counts.get(reason, 0) + int(raw)
        except ValueError:
            continue
    return counts


def _parse_center_range(payload):
    parts = str(payload or '').split(':')
    if len(parts) != 4:
        raise ValueError('selected_blob_center_range')
    return [int(value) for value in parts]


def _update_range(ranges, key, value):
    current = ranges.get(key)
    if current is None:
        ranges[key] = [value, value]
    else:
        current[0] = min(current[0], value)
        current[1] = max(current[1], value)


def _malformed(report, reason):
    report['malformed_count'] += 1
    report['malformed_reasons'][reason] = report['malformed_reasons'].get(reason, 0) + 1


def _parse_detect_stats(line, report):
    fields = line.split(',')
    if len(fields) < DETECT_STATS_FIELD_COUNT:
        _malformed(report, 'detect_stats_missing_fields')
        return
    if len(fields) > DETECT_STATS_FIELD_COUNT:
        _malformed(report, 'detect_stats_extra_fields')
        return
    if fields[DETECT_STATS_INDEX['prefix']] != 'D_DETECT_STATS':
        _malformed(report, 'detect_stats_bad_prefix')
        return
    parsed = {'mode': fields[DETECT_STATS_INDEX['mode']]}
    try:
        for name in INTEGER_FIELDS:
            parsed[name] = int(fields[DETECT_STATS_INDEX[name]])
        for name in FLOAT_FIELDS:
            parsed[name] = float(fields[DETECT_STATS_INDEX[name]])
        parsed['selected_blob_center_range'] = _parse_center_range(
            fields[DETECT_STATS_INDEX['selected_blob_center_range']])
        parsed['reject_payload'] = fields[DETECT_STATS_INDEX['reject_payload']]
    except ValueError as error:
        _malformed(report, 'detect_stats_invalid_%s' % str(error))
        return
    report['detect_stats_count'] += 1
    stage_counts = report['candidate_stage_counts']
    for name in INTEGER_FIELDS:
        if name in stage_counts:
            stage_counts[name] += parsed[name]
    _update_range(report['best_confidence_range_obj'], 'best_confidence_range', parsed['best_confidence'])
    _update_range(report['best_confidence_range_obj'], 'best_circle_ratio_range', parsed['best_ratio'])
    _update_range(report['best_confidence_range_obj'], 'best_cross_score_range', parsed['best_cross_score'])
    _update_range(report['best_confidence_range_obj'], 'track_jump_px_range', parsed['track_jump_px'])
    _update_range(report['best_confidence_range_obj'], 'track_jump_diameter_ratio_range', parsed['track_jump_diameter_ratio'])
    _update_range(report['best_confidence_range_obj'], 'selected_blob_diameter_min_range', parsed['selected_blob_diameter_min'])
    _update_range(report['best_confidence_range_obj'], 'selected_blob_diameter_max_range', parsed['selected_blob_diameter_max'])
    _update_range(report['best_confidence_range_obj'], 'expected_outer_diameter_range', parsed['expected_outer_diameter'])
    _update_range(report['best_confidence_range_obj'], 'current_blob_diameter_range', parsed['current_blob_diameter'])
    _update_range(report['best_confidence_range_obj'], 'hough_radius_min_range', parsed['hough_radius_min'])
    _update_range(report['best_confidence_range_obj'], 'hough_radius_max_range', parsed['hough_radius_max'])
    center_ranges = report['selected_blob_center_range']
    if center_ranges is None:
        report['selected_blob_center_range'] = parsed['selected_blob_center_range'][:]
    else:
        report['selected_blob_center_range'] = [
            min(center_ranges[0], parsed['selected_blob_center_range'][0]),
            min(center_ranges[1], parsed['selected_blob_center_range'][1]),
            max(center_ranges[2], parsed['selected_blob_center_range'][2]),
            max(center_ranges[3], parsed['selected_blob_center_range'][3]),
        ]
    reasons = _parse_reject_counts(parsed['reject_payload'])
    for reason, count in reasons.items():
        report['reject_reason_counts'][reason] = report['reject_reason_counts'].get(reason, 0) + count


def analyze_lines(lines, seconds):
    report = {
        'serial_line_hz': 0.0,
        'target_v2_hz': 0.0,
        'unique_sequence_count': 0,
        'new_frame_hz': 0.0,
        'valid_detection_hz': 0.0,
        'sequence_gap_count': 0,
        'duplicate_sequence_count': 0,
        'malformed_count': 0,
        'malformed_reasons': {},
        'protocol_error_count': 0,
        'processing_p50_ms': 0.0,
        'processing_p95_ms': 0.0,
        'reported_camera_fps': 0.0,
        'boot_v2_seen': False,
        'status_v2_count': 0,
        'detect_stats_count': 0,
        'candidate_stage_counts': {
            'blob_count': 0,
            'region_count': 0,
            'strong_verify_due_count': 0,
            'strong_verify_attempt_count': 0,
            'strong_verify_success_count': 0,
            'strong_verify_failure_count': 0,
            'circle_count': 0,
            'circle_pair_count': 0,
            'cross_line_count': 0,
            'cross_candidate_count': 0,
            'last_verified_age': 0,
            'current_measurement_count': 0,
            'current_valid_frame_count': 0,
            'verification_grace_frame_count': 0,
            'candidate_switch_count': 0,
            'edge_clipped_count': 0,
            'verify_roi_clamped_count': 0,
            'full_search_reset_count': 0,
            'acquire_timeout_count': 0,
            'verified_track_reset_count': 0,
        },
        'reject_reason_counts': {},
        'strong_verify_attempt_count': 0,
        'strong_verify_success_rate': 0.0,
        'runtime_config': {},
        'selected_blob_center_range': None,
        'best_confidence_range_obj': {},
    }
    target = []
    valid = 0
    vision_fps = []
    sequences = []
    for raw in lines:
        line = raw.decode('utf-8', 'replace').strip() if isinstance(raw, bytes) else str(raw).strip()
        if line.startswith('D_BOOT_V2,'):
            report['boot_v2_seen'] = True
        elif line.startswith(CONFIG_PREFIX):
            for item in line[len(CONFIG_PREFIX):].split(','):
                if '=' not in item:
                    continue
                key, value = item.split('=', 1)
                report['runtime_config'][key] = value
        elif line.startswith('D_PROTOCOL_ERROR,'):
            report['protocol_error_count'] += 1
        elif line.startswith('D_STATUS_V2,'):
            report['status_v2_count'] += 1
        elif line.startswith('D_VISION,'):
            match = re.search(r'fps=([0-9.]+)', line)
            if match:
                vision_fps.append(float(match.group(1)))
        elif line.startswith(DETECT_STATS_PREFIX):
            _parse_detect_stats(line, report)
        elif line.startswith('D_TARGET_V2,'):
            fields = line.split(',')
            if len(fields) != 12:
                _malformed(report, 'target_v2_bad_field_count')
                continue
            try:
                sequence = int(fields[1])
                processing = float(fields[3])
                is_valid = int(fields[5])
                if sequence < 0 or processing < 0 or is_valid not in (0, 1):
                    raise ValueError('target_v2_invalid_numeric')
                sequences.append(sequence)
                target.append(processing)
                valid += is_valid
            except ValueError as error:
                _malformed(report, str(error))
    unique = sorted(set(sequences))
    values = sorted(target)
    report['serial_line_hz'] = len(list(lines)) / max(seconds, 1e-9)
    report['target_v2_hz'] = len(target) / max(seconds, 1e-9)
    report['unique_sequence_count'] = len(unique)
    report['new_frame_hz'] = len(unique) / max(seconds, 1e-9)
    report['valid_detection_hz'] = valid / max(seconds, 1e-9)
    report['sequence_gap_count'] = sum(max(0, b - a - 1) for a, b in zip(unique, unique[1:]))
    report['duplicate_sequence_count'] = len(sequences) - len(unique)
    report['processing_p50_ms'] = (values[min(len(values) - 1, int(len(values) * .50))] / 1000.0) if values else 0.0
    report['processing_p95_ms'] = (values[min(len(values) - 1, int(len(values) * .95))] / 1000.0) if values else 0.0
    report['processing_p99_ms'] = (values[min(len(values) - 1, int(len(values) * .99))] / 1000.0) if values else 0.0
    report['processing_max_ms'] = (values[-1] / 1000.0) if values else 0.0
    report['reported_camera_fps'] = sum(vision_fps) / len(vision_fps) if vision_fps else 0.0
    attempts = report['candidate_stage_counts']['strong_verify_attempt_count']
    successes = report['candidate_stage_counts']['strong_verify_success_count']
    report['strong_verify_attempt_count'] = attempts
    report['strong_verify_success_rate'] = max(0.0, min(1.0, (successes / attempts) if attempts else 0.0))
    ranges = report.pop('best_confidence_range_obj')
    report['best_confidence_range'] = ranges.get('best_confidence_range', [0.0, 0.0])
    report['best_circle_ratio_range'] = ranges.get('best_circle_ratio_range', [0.0, 0.0])
    report['best_cross_score_range'] = ranges.get('best_cross_score_range', [0.0, 0.0])
    report['track_jump_px_range'] = ranges.get('track_jump_px_range', [0.0, 0.0])
    report['track_jump_diameter_ratio_range'] = ranges.get('track_jump_diameter_ratio_range', [0.0, 0.0])
    report['selected_blob_diameter_min_range'] = ranges.get('selected_blob_diameter_min_range', [0.0, 0.0])
    report['selected_blob_diameter_max_range'] = ranges.get('selected_blob_diameter_max_range', [0.0, 0.0])
    report['expected_outer_diameter_range'] = ranges.get('expected_outer_diameter_range', [0.0, 0.0])
    report['current_blob_diameter_range'] = ranges.get('current_blob_diameter_range', [0.0, 0.0])
    report['hough_radius_min_range'] = ranges.get('hough_radius_min_range', [0.0, 0.0])
    report['hough_radius_max_range'] = ranges.get('hough_radius_max_range', [0.0, 0.0])
    if not target:
        report['error'] = 'PROTOCOL_V2_MISSING'
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/dtask_openmv')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args()
    if serial is None:
        raise SystemExit('pyserial is required')
    lines = []
    with serial.Serial(args.port, 115200, timeout=.1) as port:
        started = time.monotonic()
        while time.monotonic() - started < args.seconds:
            line = port.readline()
            if line:
                lines.append(line)
    report = analyze_lines(lines, args.seconds)
    print(json.dumps(report, sort_keys=True) if args.json else report)
    if report.get('error') == 'PROTOCOL_V2_MISSING':
        print('PROTOCOL_V2_MISSING', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
