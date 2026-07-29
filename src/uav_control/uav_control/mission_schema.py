"""Central schemas and enums for the 2026 D-task mission controller."""

from enum import IntEnum


class MissionMode(IntEnum):
    DROP = 1
    DYNAMIC_LAND = 2
    HOVER_TEST = 3


MODE_NAMES = {'drop': MissionMode.DROP, 'dynamic_land': MissionMode.DYNAMIC_LAND,
              'hover_test': MissionMode.HOVER_TEST}


class CarProgress(IntEnum):
    UNKNOWN_OR_IDLE = 0
    STARTED_AT_A = 1
    PASSED_B = 2
    PASSED_C = 3
    PASSED_D = 4
    RETURNED_A = 5


STATES = (
    'WAIT_PX4', 'WAIT_SAFETY', 'WAIT_START', 'PRESTREAM',
    'REQUEST_OFFBOARD', 'WAIT_MANUAL_ARM', 'ARMING', 'TAKEOFF',
    'HOVER_CONFIRM', 'HOVER_TEST', 'SEARCH_TARGET', 'FOLLOW_TARGET',
    'DROP_ALIGN', 'DROP_RELEASE', 'LANDING_ALIGN', 'DESCEND_ON_CAR',
    'TOUCHDOWN_VERIFY', 'DISARM_ON_CAR', 'DWELL_ON_CAR',
    'SECOND_PRESTREAM', 'SECOND_ARM', 'SECOND_TAKEOFF', 'RETURN_H',
    'LAND_H', 'WAIT_DISARM', 'COMPLETE', 'ABORT_RETURN_H',
    'FAILSAFE_LAND', 'EXTERNAL_CONTROL')
STATE_ID = {name: index for index, name in enumerate(STATES)}

TELEMETRY_FIELDS = (
    'mission_active', 'mission_mode', 'state_id', 'elapsed_seconds',
    'x', 'y', 'z', 'vx', 'vy', 'vz', 'heading', 'target_valid',
    'target_error_x', 'target_error_y', 'target_confidence', 'car_progress',
    'armed', 'offboard_active', 'failsafe', 'command_ack_status',
    'touchdown_confirmed', 'h_distance')
TELEMETRY = {name: index for index, name in enumerate(TELEMETRY_FIELDS)}
TELEMETRY_LENGTH = len(TELEMETRY_FIELDS)


def parse_mission_mode(name):
    try:
        return MODE_NAMES[str(name)]
    except KeyError as error:
        raise ValueError('mission_mode must be drop, dynamic_land or hover_test') from error
