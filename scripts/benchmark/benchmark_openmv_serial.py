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


def _update_range(ranges, key, value):
    if value is None:
        return
    current = ranges.get(key)
    if current is None:
        ranges[key] = [value, value]
    else:
        current[0] = min(current[0], value)
        current[1] = max(current[1], value)


def analyze_lines(lines, seconds):
    target = []
    valid = 0
    malformed = 0
    status = 0
    errors = 0
    boot = False
    vision_fps = []
    sequences = []
    reject_reason_counts = {}
    stage_counts = {
        'blob_count': 0,
        'region_count': 0,
        'verify_due_count': 0,
        'verify_attempt_count': 0,
        'circle_count': 0,
        'circle_pair_count': 0,
        'cross_line_count': 0,
        'cross_candidate_count': 0,
        'valid_count': 0,
    }
    ranges = {
        'best_confidence_range': None,
        'best_circle_ratio_range': None,
        'best_cross_score_range': None,
    }
    runtime_config = {}
    strong_verify_successes = 0
    detect_stats_count = 0
    for raw in lines:
        line = raw.decode('utf-8', 'replace').strip() if isinstance(raw, bytes) else str(raw).strip()
        if line.startswith('D_BOOT_V2,'):
            boot = True
        elif line.startswith(CONFIG_PREFIX):
            for item in line[len(CONFIG_PREFIX):].split(','):
                if '=' not in item:
                    continue
                key, value = item.split('=', 1)
                runtime_config[key] = value
        elif line.startswith('D_PROTOCOL_ERROR,'):
            errors += 1
        elif line.startswith('D_STATUS_V2,'):
            status += 1
        elif line.startswith('D_VISION,'):
            match = re.search(r'fps=([0-9.]+)', line)
            if match:
                vision_fps.append(float(match.group(1)))
        elif line.startswith(DETECT_STATS_PREFIX):
            fields = line.split(',', 16)
            if len(fields) != 17:
                malformed += 1
                continue
            detect_stats_count += 1
            try:
                stage_counts['blob_count'] += int(fields[3])
                stage_counts['region_count'] += int(fields[4])
                stage_counts['verify_due_count'] += int(fields[5])
                stage_counts['verify_attempt_count'] += int(fields[6])
                stage_counts['circle_count'] += int(fields[7])
                stage_counts['circle_pair_count'] += int(fields[8])
                stage_counts['cross_line_count'] += int(fields[9])
                _update_range(ranges, 'best_cross_score_range', float(fields[10]))
                _update_range(ranges, 'best_confidence_range', float(fields[11]))
                _update_range(ranges, 'best_circle_ratio_range', float(fields[12]))
                stage_counts['cross_candidate_count'] += int(fields[13])
                stage_counts['valid_count'] += int(fields[15])
                reasons = _parse_reject_counts(fields[16])
                for reason, count in reasons.items():
                    reject_reason_counts[reason] = reject_reason_counts.get(reason, 0) + count
                strong_verify_successes += reasons.get('VALID', 0)
            except (TypeError, ValueError):
                malformed += 1
        elif line.startswith('D_TARGET_V2,'):
            fields = line.split(',')
            if len(fields) != 12:
                malformed += 1
                continue
            try:
                sequence = int(fields[1])
                processing = float(fields[3])
                is_valid = int(fields[5])
                if sequence < 0 or processing < 0 or is_valid not in (0, 1):
                    raise ValueError
                sequences.append(sequence)
                target.append(processing)
                valid += is_valid
            except (TypeError, ValueError):
                malformed += 1
    unique = sorted(set(sequences))
    duplicates = len(sequences) - len(unique)
    gaps = sum(max(0, b - a - 1) for a, b in zip(unique, unique[1:]))
    target_hz = len(target) / max(seconds, 1e-9)
    values = sorted(target)
    p50 = values[min(len(values) - 1, int(len(values) * .50))] if values else 0.0
    p95 = values[min(len(values) - 1, int(len(values) * .95))] if values else 0.0
    attempts = stage_counts['verify_attempt_count']
    report = {
        'serial_line_hz': len(list(lines)) / max(seconds, 1e-9),
        'target_v2_hz': target_hz,
        'unique_sequence_count': len(unique),
        'new_frame_hz': len(unique) / max(seconds, 1e-9),
        'valid_detection_hz': valid / max(seconds, 1e-9),
        'sequence_gap_count': gaps,
        'duplicate_sequence_count': duplicates,
        'malformed_count': malformed,
        'protocol_error_count': errors,
        'processing_p50_ms': p50 / 1000.0,
        'processing_p95_ms': p95 / 1000.0,
        'reported_camera_fps': sum(vision_fps) / len(vision_fps) if vision_fps else 0.0,
        'boot_v2_seen': boot,
        'status_v2_count': status,
        'detect_stats_count': detect_stats_count,
        'candidate_stage_counts': stage_counts,
        'reject_reason_counts': reject_reason_counts,
        'strong_verify_attempt_count': attempts,
        'strong_verify_success_rate': (strong_verify_successes / attempts) if attempts else 0.0,
        'best_confidence_range': ranges['best_confidence_range'] or [0.0, 0.0],
        'best_circle_ratio_range': ranges['best_circle_ratio_range'] or [0.0, 0.0],
        'best_cross_score_range': ranges['best_cross_score_range'] or [0.0, 0.0],
        'runtime_config': runtime_config,
    }
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
