"""Pure helpers for the 3x3 SITL hover matrix and result summaries."""

import csv
import json
import math
from pathlib import Path


HEIGHTS_M = (0.5, 1.0, 1.5)
HOVER_DURATIONS_S = (5.0, 10.0, 15.0)
RESULT_FIELDS = (
    'target_height_m', 'commanded_hover_duration_s',
    'stable_hover_duration_s', 'hover_stability_passed',
    'mission_elapsed_s',
    'mean_hover_height_m', 'max_hover_height_m', 'height_error_m',
    'initial_disarmed', 'auto_disarmed', 'failsafe', 'final_state',
    'passed', 'reason')


def failure_result(scenario, target_height_m, commanded_hover_duration_s,
                   reason, final_state='NOT_STARTED'):
    """Build the canonical result for a run rejected before flight."""
    return {
        'scenario': scenario,
        'target_height_m': (
            None if target_height_m is None else float(target_height_m)),
        'commanded_hover_duration_s': (
            None if commanded_hover_duration_s is None else
            float(commanded_hover_duration_s)),
        'stable_hover_duration_s': 0.0,
        'hover_stability_passed': False,
        'mission_elapsed_s': 0.0,
        'hover_duration_s': 0.0,
        'mean_hover_height_m': None,
        'max_hover_height_m': None,
        'height_error_m': None,
        'initial_disarmed': None,
        'auto_disarmed': False,
        'failsafe': None,
        'final_state': final_state,
        'passed': False,
        'reason': str(reason),
    }


def record_mode_recovery(path, passed, reason, details=None):
    """Attach post-flight mode recovery status to an existing run result."""
    destination = Path(path)
    result = json.loads(destination.read_text(encoding='utf-8'))
    result['mode_recovery_passed'] = bool(passed)
    result['mode_recovery_reason'] = str(reason)
    if details:
        for field in (
                'mode_recovery_ack', 'final_arming_state',
                'final_nav_state', 'final_nav_state_user_intention',
                'final_failsafe', 'final_landed',
                'final_pre_flight_checks_pass'):
            if field in details:
                result[field] = details[field]
    if not passed:
        result['passed'] = False
        result['reason'] = (
            str(result.get('reason', '')).rstrip('; ') +
            '; post-flight mode recovery failed: ' + str(reason)).lstrip('; ')
    destination.write_text(
        json.dumps(result, indent=2) + '\n', encoding='utf-8')
    return result


def mode_recovery_safe(armed, landed, status_fresh, land_fresh):
    """Return true only when a post-flight mode command is safe to issue."""
    return bool(
        not armed and landed and status_fresh and land_fresh)


def mode_recovery_status_valid(
        ack_result, accepted_result, arming_state, disarmed_value, nav_state,
        nav_state_user_intention, failsafe, landed, land_fresh,
        pre_flight_checks_pass=None):
    """Validate post-flight mode state; preflight is recorded, not required."""
    del pre_flight_checks_pass
    return bool(
        ack_result == accepted_result and
        arming_state == disarmed_value and
        nav_state != 14 and
        nav_state_user_intention != 14 and
        not failsafe and landed and land_fresh)


class PreflightStability:
    """Require fresh, increasing PX4 timestamps and continuous true samples."""

    def __init__(self, stable_seconds, max_message_age_seconds):
        self.stable_seconds = float(stable_seconds)
        self.max_message_age_seconds = float(max_message_age_seconds)
        self.last_source_timestamp = None
        self.last_received_at = None
        self.true_since = None
        self.last = None
        self.reason = 'no VehicleStatus received'

    def update(self, source_timestamp, preflight_ok, received_at):
        """Consume one sample and return whether stability is satisfied."""
        source_timestamp = int(source_timestamp)
        received_at = float(received_at)
        if (self.last_received_at is not None and
                received_at - self.last_received_at >
                self.max_message_age_seconds):
            self.true_since = None
        if (self.last_source_timestamp is not None and
                source_timestamp <= self.last_source_timestamp):
            self.true_since = None
            self.reason = 'VehicleStatus timestamp did not increase'
            return False
        self.last_source_timestamp = source_timestamp
        self.last_received_at = received_at
        self.last = {
            'timestamp': source_timestamp,
            'pre_flight_checks_pass': bool(preflight_ok),
        }
        if not preflight_ok:
            self.true_since = None
            self.reason = 'pre_flight_checks_pass is false'
            return False
        if self.true_since is None:
            self.true_since = received_at
        self.reason = 'waiting for continuous preflight stability'
        return received_at - self.true_since >= self.stable_seconds

    def expired(self, now):
        """Invalidate a stream whose most recent sample is no longer fresh."""
        if (self.last_received_at is None or
                float(now) - self.last_received_at >
                self.max_message_age_seconds):
            self.true_since = None
            self.reason = 'VehicleStatus stream is stale'
            return True
        return False


class HoverMetrics:
    """Track unambiguous takeoff, stable-hover and legacy durations."""

    def __init__(self, target_height_m, altitude_tolerance_m):
        self.target_height_m = float(target_height_m)
        self.altitude_tolerance_m = float(altitude_tolerance_m)
        self.takeoff_started = None
        self.hover_started = None
        self.hover_ended = None
        self.stable_started = None
        self.max_stable_duration = 0.0
        self.samples = []

    def update_state(self, previous, current, now):
        """Record state boundaries using recorder-relative seconds."""
        now = float(now)
        if current == 'TAKEOFF' and self.takeoff_started is None:
            self.takeoff_started = now
        if current == 'HOVER_150CM' and previous != current:
            self.hover_started = now
            # Entry is only possible after MissionLogic has continuously
            # satisfied its existing altitude and velocity stability gate.
            self.stable_started = now
        if previous == 'HOVER_150CM' and current != previous:
            self.hover_ended = now
            self.close_stable(now)

    def update_height(self, state, height_m, now):
        """Update hover samples and continuous in-tolerance duration."""
        if state != 'HOVER_150CM' or height_m is None:
            self.close_stable(now)
            return
        height = float(height_m)
        if not math.isfinite(height):
            self.close_stable(now)
            return
        self.samples.append(height)
        if abs(height - self.target_height_m) <= self.altitude_tolerance_m:
            if self.stable_started is None:
                self.stable_started = float(now)
        else:
            self.close_stable(now)

    def close_stable(self, now):
        """Close the current continuous in-tolerance segment."""
        if self.stable_started is None:
            return
        duration = max(0.0, float(now) - self.stable_started)
        self.max_stable_duration = max(
            self.max_stable_duration, duration)
        self.stable_started = None

    def finish(self, now, commanded_hover_duration_s=None):
        """Return metrics whose start/end semantics are explicit."""
        now = float(now)
        self.close_stable(now)
        mean_height = (
            sum(self.samples) / len(self.samples)
            if self.samples else None)
        commanded = (
            None if commanded_hover_duration_s is None
            else float(commanded_hover_duration_s))
        return {
            'stable_hover_duration_s': self.max_stable_duration,
            'hover_stability_passed': (
                None if commanded is None else
                self.max_stable_duration >= commanded),
            'mission_elapsed_s': (
                now - self.takeoff_started
                if self.takeoff_started is not None else 0.0),
            'hover_duration_s': (
                now - self.hover_started
                if self.hover_started is not None else 0.0),
            'mean_hover_height_m': mean_height,
            'max_hover_height_m': (
                max(self.samples) if self.samples else None),
            'height_error_m': (
                abs(mean_height - self.target_height_m)
                if mean_height is not None else None),
        }


def matrix_cases():
    """Return the fixed height-major 3x3 matrix."""
    return tuple(
        (height, duration)
        for height in HEIGHTS_M
        for duration in HOVER_DURATIONS_S)


def result_passed(result):
    """Return whether a result permits the matrix to continue."""
    return bool(
        result.get('passed')
        and result.get('final_state') == 'COMPLETE'
        and result.get('initial_disarmed')
        and result.get('auto_disarmed')
        and result.get('failsafe') is False)


def run_until_failure(cases, runner):
    """Run cases in order and stop immediately after the first failure."""
    results = []
    for case in cases:
        result = runner(*case)
        results.append(result)
        if not result_passed(result):
            break
    return results


def load_json_lines(path):
    """Load non-empty JSON lines from a result stream."""
    lines = Path(path).read_text(encoding='utf-8').splitlines()
    return [json.loads(line) for line in lines if line.strip()]


def write_summary(results, csv_path, json_path):
    """Write the matrix summary in both CSV and JSON formats."""
    csv_destination = Path(csv_path)
    json_destination = Path(json_path)
    csv_destination.parent.mkdir(parents=True, exist_ok=True)
    json_destination.parent.mkdir(parents=True, exist_ok=True)
    with csv_destination.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=RESULT_FIELDS)
        writer.writeheader()
        for result in results:
            writer.writerow({
                field: result.get(field) for field in RESULT_FIELDS})
    json_destination.write_text(
        json.dumps(results, indent=2) + '\n', encoding='utf-8')


def print_table(results):
    """Print a compact nine-case terminal summary."""
    header = (
        'height  command  stable  mission  mean_h  max_h  state     pass')
    print(header)
    print('-' * len(header))
    for result in results:
        values = (
            result.get('target_height_m'),
            result.get('commanded_hover_duration_s'),
            result.get('stable_hover_duration_s'),
            result.get('mission_elapsed_s'),
            result.get('mean_hover_height_m'),
            result.get('max_hover_height_m'),
        )
        formatted = [
            'n/a' if value is None else f'{float(value):.2f}'
            for value in values]
        print(
            f'{formatted[0]:>6}  {formatted[1]:>7}  '
            f'{formatted[2]:>6}  {formatted[3]:>7}  '
            f'{formatted[4]:>6}  {formatted[5]:>5}  '
            f'{str(result.get("final_state")):<9} '
            f'{str(bool(result.get("passed"))):<5}')
