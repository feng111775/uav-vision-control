"""Executable documentation for the existing D-task mission states."""

from dataclasses import dataclass


@dataclass(frozen=True)
class StageSpec:
    entry: str
    action: str
    success: str
    timeout: str
    failure: str


STAGES = {
    'WAIT_PX4': StageSpec(
        'fresh valid PX4 position and attitude are not yet established',
        'monitor only; publish no PX4 command or setpoint',
        'PX4 position and attitude are fresh and valid',
        'remain WAIT_PX4 before mission start',
        'WAIT_PX4'),
    'WAIT_SAFETY': StageSpec(
        'PX4 inputs are valid',
        'wait for simulation mode or fresh hardware safety permission',
        'SITL mode, or hardware safety and attitude are valid',
        'remain WAIT_SAFETY',
        'WAIT_SAFETY'),
    'WAIT_START': StageSpec(
        'PX4 and safety gates passed',
        'wait for car start; hover_test may use explicit control enable',
        'start requested and stationary home pose locked',
        'remain WAIT_START',
        'WAIT_START'),
    'PRESTREAM': StageSpec(
        'mission started and home pose locked',
        'stream Offboard heartbeat and home/altitude setpoint',
        'configured prestream cycle count reached',
        'DATA_TIMEOUT on critical PX4 input loss',
        'DATA_TIMEOUT'),
    'REQUEST_OFFBOARD': StageSpec(
        'prestream complete',
        'request PX4 Offboard with finite retry/ack tracking',
        'PX4 reports NAVIGATION_STATE_OFFBOARD',
        'FAILSAFE after command retry exhaustion',
        'FAILSAFE'),
    'ARMING': StageSpec(
        'Offboard active and automatic arm policy allows it',
        'SITL/non-hover policy may request ARM; hover hardware never does',
        'PX4 reports Armed while still Offboard',
        'FAILSAFE after command retry exhaustion',
        'FAILSAFE'),
    'WAIT_MANUAL_ARM': StageSpec(
        'automatic arm policy is disabled or hardware hover_test is active',
        'keep valid Offboard setpoints; never issue ARM',
        'operator arms and PX4 reports Armed plus Offboard',
        'DATA_TIMEOUT on critical PX4 input loss',
        'DATA_TIMEOUT'),
    'TAKEOFF': StageSpec(
        'Armed and Offboard',
        'hold home XY and command configured NED cruise altitude',
        'altitude and velocity stable for stable_seconds',
        'mission deadline routes to RETURN_HOME',
        'FAILSAFE or RETURN_HOME'),
    'HOVER_150CM': StageSpec(
        'takeoff altitude stable',
        'hold configured altitude (state name is legacy, value is parameterized)',
        'hover_test timer expires, or non-hover advances to HOVER_3S',
        'mission deadline routes to RETURN_HOME',
        'FAILSAFE or RETURN_HOME'),
    'HOVER_3S': StageSpec(
        'non-hover initial hover reached',
        'hold home and cruise altitude',
        'at least max(3 s, hover_confirm_seconds)',
        'mission deadline routes to RETURN_HOME',
        'FAILSAFE or RETURN_HOME'),
    'SEARCH_CAR': StageSpec(
        'initial hover confirmed',
        'hold position and wait for stable vision target',
        'target valid continuously for visual_stable_seconds',
        'D passed or mission deadline routes to RETURN_HOME',
        'RETURN_HOME'),
    'VISION_FOLLOW': StageSpec(
        'stable target acquired',
        'apply bounded horizontal visual guidance at cruise altitude',
        'target alignment stable',
        'target loss holds current position; deadline returns home',
        'RETURN_HOME'),
    'ALIGN_FOR_DROP': StageSpec(
        'drop mode target alignment reached',
        'maintain alignment and cruise altitude',
        'aligned, altitude reached, before D, payload release enabled',
        'D passed or mission deadline routes to RETURN_HOME',
        'RETURN_HOME'),
    'PAYLOAD_RELEASE': StageSpec(
        'drop conditions satisfied exactly once',
        'publish one payload release pulse through the adapter',
        'advance to WAIT_RELEASE_ACK',
        'immediate bounded transition',
        'WAIT_RELEASE_ACK'),
    'WAIT_RELEASE_ACK': StageSpec(
        'payload pulse emitted',
        'wait for a new post-release acknowledgement',
        'ack received, then RETURN_HOME',
        'payload_ack_timeout, then RETURN_HOME',
        'RETURN_HOME'),
    'ALIGN_PLATFORM': StageSpec(
        'dynamic-land target alignment reached',
        'maintain visual alignment at cruise altitude',
        'alignment stable, landing enabled, car has not passed D',
        'D passed or mission deadline routes to RETURN_HOME',
        'RETURN_HOME'),
    'DYNAMIC_DESCENT_HIGH': StageSpec(
        'platform aligned',
        'bounded high descent only while target remains stable',
        'near-height threshold or touchdown candidate',
        'car/PX4/vision loss aborts descent',
        'RETURN_HOME or FAILSAFE'),
    'DYNAMIC_DESCENT_NEAR': StageSpec(
        'near-height threshold reached',
        'bounded slow descent only while target remains stable',
        'touchdown candidate',
        'car/PX4/vision loss aborts descent',
        'RETURN_HOME or FAILSAFE'),
    'TOUCHDOWN_CHECK': StageSpec(
        'touchdown candidate detected',
        'use bounded contact descent and continuous touchdown validation',
        'touchdown confirmed',
        'candidate loss returns to DYNAMIC_DESCENT_NEAR',
        'DYNAMIC_DESCENT_NEAR or RETURN_HOME'),
    'LANDED_ON_CAR': StageSpec(
        'touchdown confirmed before D',
        'latch touchdown result',
        'advance to DWELL_5S',
        'immediate bounded transition',
        'FAILSAFE'),
    'DWELL_5S': StageSpec(
        'landed on moving platform',
        'maintain bounded contact command in SITL',
        'at least configured dwell time (minimum 5 s)',
        'mission deadline routes to RETURN_HOME',
        'FAILSAFE'),
    'DISARM_ON_CAR': StageSpec(
        'dwell complete',
        'request PX4 land/disarm only under touchdown gates',
        'PX4 reports Disarmed',
        'command retry exhaustion enters FAILSAFE',
        'FAILSAFE'),
    'SECOND_PRESTREAM': StageSpec(
        'optional second flight enabled and disarm confirmed',
        'prestream only after fresh PX4 and stable visual target',
        'full prestream count reached',
        'remain until prerequisites recover',
        'DATA_TIMEOUT'),
    'SECOND_ARM': StageSpec(
        'second Offboard request accepted',
        'request ARM only through existing policy',
        'PX4 reports Armed and Offboard',
        'command retry exhaustion enters FAILSAFE',
        'FAILSAFE'),
    'SECOND_TAKEOFF': StageSpec(
        'second arm confirmed',
        'command configured cruise altitude',
        'altitude and velocity stable',
        'mission deadline routes to RETURN_HOME',
        'RETURN_HOME or FAILSAFE'),
    'RETURN_HOME': StageSpec(
        'task complete, aborted, D passed, or deadline reached',
        'command home XY at cruise altitude',
        'home position and speed tolerance reached',
        'critical fault enters FAILSAFE',
        'FAILSAFE'),
    'FINAL_LAND': StageSpec(
        'home reached or hover_test timer expired',
        'request PX4 NAV_LAND with bounded acknowledgement tracking',
        'PX4 reports Disarmed',
        'command retry exhaustion enters FAILSAFE',
        'FAILSAFE'),
    'COMPLETE': StageSpec(
        'final Disarmed confirmation received',
        'publish observability only; no PX4 output',
        'operator may request safe reset',
        'none',
        'COMPLETE'),
    'DATA_TIMEOUT': StageSpec(
        'critical PX4 status, position, or attitude became stale',
        'block further arming and task progression',
        'requires safe operator reset/restart',
        'terminal for the current mission',
        'DATA_TIMEOUT'),
    'FAILSAFE': StageSpec(
        'PX4 failsafe, tilt, Offboard exit, command failure, or regression',
        'block further arming and task progression',
        'requires PX4/operator recovery and safe reset',
        'terminal for the current mission',
        'FAILSAFE'),
}


SAFE_STAGE_TARGETS = frozenset(STAGES)


def validate_stage_target(stage):
    """Validate an observer target; this never changes controller state."""
    if stage not in SAFE_STAGE_TARGETS:
        raise ValueError('unknown D-task stage: ' + str(stage))
    return stage
