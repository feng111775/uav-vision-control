"""Central schemas and enums for the 2026 D-task mission controller."""

from enum import Enum, IntEnum


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


class MissionState(str, Enum):
    WAIT_PX4 = 'WAIT_PX4'
    WAIT_SAFETY = 'WAIT_SAFETY'
    WAIT_START = 'WAIT_START'
    PRESTREAM = 'PRESTREAM'
    REQUEST_OFFBOARD = 'REQUEST_OFFBOARD'
    ARMING = 'ARMING'
    WAIT_MANUAL_ARM = 'WAIT_MANUAL_ARM'
    TAKEOFF = 'TAKEOFF'
    HOVER_150CM = 'HOVER_150CM'
    HOVER_3S = 'HOVER_3S'
    SEARCH_CAR = 'SEARCH_CAR'
    VISION_FOLLOW = 'VISION_FOLLOW'
    ALIGN_FOR_DROP = 'ALIGN_FOR_DROP'
    PAYLOAD_RELEASE = 'PAYLOAD_RELEASE'
    WAIT_RELEASE_ACK = 'WAIT_RELEASE_ACK'
    ALIGN_PLATFORM = 'ALIGN_PLATFORM'
    DYNAMIC_DESCENT_HIGH = 'DYNAMIC_DESCENT_HIGH'
    DYNAMIC_DESCENT_NEAR = 'DYNAMIC_DESCENT_NEAR'
    TOUCHDOWN_CHECK = 'TOUCHDOWN_CHECK'
    LANDED_ON_CAR = 'LANDED_ON_CAR'
    DWELL_5S = 'DWELL_5S'
    DISARM_ON_CAR = 'DISARM_ON_CAR'
    SECOND_PRESTREAM = 'SECOND_PRESTREAM'
    SECOND_ARM = 'SECOND_ARM'
    SECOND_TAKEOFF = 'SECOND_TAKEOFF'
    RETURN_HOME = 'RETURN_HOME'
    FINAL_LAND = 'FINAL_LAND'
    COMPLETE = 'COMPLETE'
    DATA_TIMEOUT = 'DATA_TIMEOUT'
    FAILSAFE = 'FAILSAFE'


STATES = tuple(state.value for state in MissionState)
STATE_ID = {name: index for index, name in enumerate(STATES)}

TELEMETRY_FIELDS = (
    'mission_active', 'mission_mode', 'state_id', 'elapsed_seconds',
    'state_elapsed_seconds', 'remaining_seconds',
    'x', 'y', 'z', 'vx', 'vy', 'vz', 'heading',
    'relative_h_height', 'h_distance',
    'target_valid', 'vision_age_ms', 'target_error_x', 'target_error_y',
    'target_confidence', 'car_progress', 'formed_follow_before_b',
    'completed_before_d', 'armed', 'nav_state', 'offboard_active',
    'failsafe', 'px4_fresh', 'payload_sent', 'payload_ack',
    'touchdown_candidate', 'touchdown_confirmed', 'dwell_progress',
    'command_ack_status', 'safety_block_code')
TELEMETRY = {name: index for index, name in enumerate(TELEMETRY_FIELDS)}
TELEMETRY_LENGTH = len(TELEMETRY_FIELDS)

SAFETY_BLOCK_CODES = {
    'NONE': 0,
    'WAIT_PX4': 1,
    'WAIT_SAFETY': 2,
    'WAIT_START': 3,
    'PX4_DATA_TIMEOUT': 4,
    'PX4_FAILSAFE': 5,
    'ABNORMAL_TILT': 6,
    'OFFBOARD_EXIT': 7,
    'VISION_INVALID': 8,
    'CAR_PROGRESS_REGRESSION': 9,
    'D_PASSED_ABORT': 10,
    'MISSION_DEADLINE_RETURN': 11,
    'PHYSICAL_TOUCHDOWN_REQUIRED': 12,
    'AUTO_DISARM_DISABLED': 13,
    'MISSION_ABORT_REQUESTED': 14,
    'PAYLOAD_ACK_TIMEOUT': 15,
    'PAYLOAD_ACK_FAILED': 16,
}
SAFETY_BLOCK_NAMES = {value: name for name, value in SAFETY_BLOCK_CODES.items()}


def parse_mission_mode(name):
    try:
        return MODE_NAMES[str(name)]
    except KeyError as error:
        raise ValueError('mission_mode must be drop, dynamic_land or hover_test') from error
