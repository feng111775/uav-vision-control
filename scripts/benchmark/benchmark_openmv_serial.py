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


def analyze_lines(lines, seconds):
    target = []; valid = 0; malformed = 0; status = 0; errors = 0
    boot = False; vision_fps = []; sequences = []
    for raw in lines:
        line = raw.decode('utf-8', 'replace').strip() if isinstance(raw, bytes) else str(raw).strip()
        if line.startswith('D_BOOT_V2,'): boot = True
        elif line.startswith('D_PROTOCOL_ERROR,'): errors += 1
        elif line.startswith('D_STATUS_V2,'): status += 1
        elif line.startswith('D_VISION,'):
            match = re.search(r'fps=([0-9.]+)', line)
            if match: vision_fps.append(float(match.group(1)))
        elif line.startswith('D_TARGET_V2,'):
            fields = line.split(',')
            if len(fields) != 12:
                malformed += 1; continue
            try:
                sequence = int(fields[1]); processing = float(fields[3]); is_valid = int(fields[5])
                if sequence < 0 or processing < 0 or is_valid not in (0, 1): raise ValueError
                sequences.append(sequence); target.append(processing)
                valid += is_valid
            except (TypeError, ValueError): malformed += 1
    unique = sorted(set(sequences)); duplicates = len(sequences) - len(unique)
    gaps = sum(max(0, b - a - 1) for a, b in zip(unique, unique[1:]))
    target_hz = len(target) / max(seconds, 1e-9)
    values = sorted(target)
    p50 = values[min(len(values)-1, int(len(values)*.50))] if values else 0.0
    p95 = values[min(len(values)-1, int(len(values)*.95))] if values else 0.0
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
            if line: lines.append(line)
    report = analyze_lines(lines, args.seconds)
    print(json.dumps(report, sort_keys=True) if args.json else report)
    if report.get('error') == 'PROTOCOL_V2_MISSING':
        print('PROTOCOL_V2_MISSING', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
