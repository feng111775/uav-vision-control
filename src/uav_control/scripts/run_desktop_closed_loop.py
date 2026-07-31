#!/usr/bin/env python3
"""Run a PX4-free deterministic mission-logic desktop acceptance."""

import json
from pathlib import Path

from uav_control.mission_logic import MissionLogic


def run():
    logic = MissionLogic(
        'drop', simulation_mode=True, enable_control=True,
        enable_auto_arm=True, enable_payload_release=True,
        prestream_seconds=0.1, stable_seconds=0.1,
        hover_confirm_seconds=3.0, visual_stable_seconds=0.4)
    states = [logic.state]
    now = 0.0

    def tick(value, start=False, armed=None, offboard=None,
             visual=None, ack=False):
        nonlocal now
        now = float(value)
        if armed is not None or offboard is not None:
            logic.update_status(
                logic.armed if armed is None else armed,
                logic.offboard if offboard is None else offboard, False)
        if visual is not None:
            logic.update_visual(visual[0], visual[1], now)
        if start:
            logic.start_signal = True
        if ack:
            logic.payload_ack = True
        logic.step(now)
        if not states or states[-1] != logic.state:
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
    # Altitude stability transitions into the three-second hover confirmation.
    tick(3.55, armed=True, offboard=True)
    tick(6.55, armed=True, offboard=True)
    tick(6.56, armed=True, offboard=True, visual=(True, False))
    tick(6.57, armed=True, offboard=True, visual=(True, True))
    tick(6.58, armed=True, offboard=True, visual=(True, True))
    # The first 0.39 s must not request release.
    tick(6.97, armed=True, offboard=True, visual=(True, True))
    release_before = int(logic.payload_sent)
    # Break and restart the confirmation window.
    tick(6.98, armed=True, offboard=True, visual=(False, False))
    tick(7.00, armed=True, offboard=True, visual=(True, True))
    tick(7.39, armed=True, offboard=True, visual=(True, True))
    tick(7.41, armed=True, offboard=True, visual=(True, True))
    release_after = int(logic.payload_sent)
    tick(7.42, armed=True, offboard=True, ack=True)
    logic.update_position(0, 0, logic.cruise_z, 0, 0, 0, 0, True)
    tick(7.52, armed=True, offboard=True)
    tick(7.53, armed=False, offboard=True)
    tick(7.54, armed=False, offboard=True)
    passed = (states == [
        'WAIT_PX4', 'WAIT_SAFETY', 'WAIT_START', 'PRESTREAM',
        'REQUEST_OFFBOARD', 'ARMING', 'TAKEOFF', 'HOVER_CONFIRM',
        'SEARCH_TARGET', 'FOLLOW_TARGET', 'DROP_ALIGN', 'DROP_RELEASE',
        'RETURN_H', 'LAND_H', 'WAIT_DISARM', 'COMPLETE'])
    result = {
        'passed': bool(passed and release_before == 0 and release_after == 1),
        'final_state': logic.state,
        'state_sequence': states,
        'hover_confirm_duration_s': 3.0,
        'align_continuous_duration_s': 0.4,
        'release_request_count': release_after,
        'release_ack_received': bool(logic.payload_ack),
        'px4_input_topics_published': [],
        'remaining_nodes': [],
        'failure_reason': '' if passed else 'state_or_release_gate_mismatch',
    }
    output = Path('artifacts/desktop_closed_loop_result.json')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(run())
