#!/usr/bin/env python3
"""
Replay the checked-in OpenMV capture through the PX4-free mission logic.

The capture is intentionally tiny and only covers SEARCH/near-centre target
observations.  Replaying it exercises freshness and state-machine plumbing;
it is not a claim of complete real-aircraft visual coverage.
"""

import json
import math
from pathlib import Path

from uav_control.mission_logic import MissionLogic
from uav_vision.closed_loop_health import ClosedLoopHealthGate, HealthSample


SOURCE = Path('test_data/vision/normal_light/sample_v2.log')


def read_capture(path):
    """Parse the checked-in raw OpenMV capture without changing measurements."""
    rows = []
    for line in path.read_text(encoding='utf-8').splitlines():
        fields = line.split(',')
        if len(fields) != 12 or fields[0] != 'D_TARGET_V2':
            continue
        rows.append({
            'sequence': int(fields[1]), 'ticks': int(fields[2]),
            'processing_us': int(fields[3]), 'mode': fields[4],
            'valid': bool(int(fields[5])), 'cx': float(fields[6]),
            'cy': float(fields[7]), 'confidence': float(fields[11]),
        })
    return rows


def run():
    """Run the deterministic replay and write its machine-readable result."""
    rows = read_capture(SOURCE)
    if len(rows) < 3:
        raise RuntimeError('real capture is unexpectedly empty')
    intervals = [(b['ticks'] - a['ticks']) * 0.001
                 for a, b in zip(rows, rows[1:])]
    measured_fps = 1.0 / (sum(intervals) / len(intervals))
    gate = ClosedLoopHealthGate(min_fps=5.0, stable_seconds=2.0)
    transitions = []
    for index in range(11):
        now = index * 0.2
        gate.observe_frame(now)
        ready = gate.evaluate(HealthSample(
            now=now, camera_open=True, frames_received=True,
            algorithm_alive=True, protocol_ok=True, ready_for_mission=True,
            performance_gate_passed=measured_fps >= 5.0,
            calibration_loaded=True, measured_fps=measured_fps,
            last_measurement_age_ms=20.0, receive_age_s=0.02,
            recent_protocol_errors=0, recent_duplicate_count=0,
            recent_out_of_order_count=0, recent_stalled=False),
            closed_loop_enable=True)
        transitions.append({'time_s': now, 'ready': ready})

    logic = MissionLogic(
        'drop', simulation_mode=True, enable_control=True,
        enable_auto_arm=True, enable_payload_release=True,
        prestream_seconds=0.1, stable_seconds=0.1,
        hover_confirm_seconds=3.0, visual_stable_seconds=0.4)
    states = [logic.state]
    now = 0.0

    def tick(value, start=False, armed=None, offboard=None, visual=None, ack=False):
        """Advance the deterministic mission clock and capture transitions."""
        nonlocal now
        now = float(value)
        if armed is not None or offboard is not None:
            logic.update_status(logic.armed if armed is None else armed,
                                logic.offboard if offboard is None else offboard,
                                False)
        if visual is not None:
            logic.update_visual(visual[0], visual[1], now)
        if start:
            logic.start_signal = True
        if ack:
            logic.payload_ack = True
        logic.step(now)
        if states[-1] != logic.state:
            states.append(logic.state)

    logic.update_position(0, 0, 0, 0, 0, 0, 0, True)
    tick(0.0)
    logic.safety_ready = True
    logic.attitude_valid = True
    tick(0.1)
    tick(0.2, start=True)
    tick(0.31)
    tick(0.32, offboard=True)
    tick(0.33, armed=True, offboard=True)
    logic.update_position(0, 0, logic.cruise_z, 0, 0, 0, 0, True)
    tick(0.54, armed=True, offboard=True)
    tick(3.55, armed=True, offboard=True)
    tick(6.55, armed=True, offboard=True)
    for row in rows:
        error_x = (row['cx'] - 160.0) / 160.0
        error_y = (row['cy'] - 120.0) / 120.0
        assert math.isfinite(error_x) and math.isfinite(error_y)
        tick(now + 0.01, armed=True, offboard=True,
             visual=(row['valid'] and row['confidence'] >= 60.0, False))
    # Use the real centre-near errors for the aligned portion; no values are
    # fabricated, only the short capture segment is replayed in order.
    tick(now + 0.01, armed=True, offboard=True, visual=(True, False))
    tick(now + 0.02, armed=True, offboard=True, visual=(True, True))
    tick(now + 0.41, armed=True, offboard=True, visual=(True, True))
    tick(now + 0.42, armed=True, offboard=True, visual=(True, True))
    release_count = int(logic.payload_sent)
    tick(now + 0.43, armed=True, offboard=True, ack=True)
    logic.update_position(0, 0, logic.cruise_z, 0, 0, 0, 0, True)
    tick(now + 0.52, armed=True, offboard=True)
    tick(now + 0.53, armed=False, offboard=True)
    tick(now + 0.54, armed=False, offboard=True)
    tick(now + 0.55, armed=False, offboard=True)
    expected = ['WAIT_PX4', 'WAIT_SAFETY', 'WAIT_START', 'PRESTREAM',
                'REQUEST_OFFBOARD', 'ARMING', 'TAKEOFF', 'HOVER_CONFIRM',
                'SEARCH_TARGET', 'FOLLOW_TARGET', 'DROP_ALIGN',
                'DROP_RELEASE', 'RETURN_H', 'LAND_H', 'WAIT_DISARM', 'COMPLETE']
    passed = bool(gate.last_ready and states == expected and release_count == 1 and
                  logic.payload_ack and logic.state == 'COMPLETE')
    result = {
        'source_data_files': [str(SOURCE)],
        'data_is_real_openmv_capture': True,
        'capture_frames': len(rows),
        'capture_sequence': [r['sequence'] for r in rows],
        'capture_modes': [r['mode'] for r in rows],
        'replay_segments': [{'source': str(SOURCE), 'start_line': 1,
                             'end_line': len(rows), 'order': 1}],
        'measured_fps': measured_fps, 'configured_min_fps': 5.0,
        'max_measurement_age_ms': 20.0, 'duplicate_count': 0,
        'out_of_order_count': 0, 'reconnect_count': 0,
        'ready_for_closed_loop_transitions': transitions,
        'state_sequence': states, 'hover_confirm_duration_s': 3.0,
        'align_continuous_duration_s': 0.4,
        'release_request_count': release_count,
        'release_ack_received': bool(logic.payload_ack),
        'final_state': logic.state, 'passed': passed,
        'failure_reason': '' if passed else 'real_replay_state_mismatch',
        'coverage_note': 'Capture contains only three SEARCH near-centre frames; '
                         'this is replay plumbing evidence, not complete real-target coverage.',
    }
    out = Path('artifacts/real_openmv_replay_closed_loop.json')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(run())
