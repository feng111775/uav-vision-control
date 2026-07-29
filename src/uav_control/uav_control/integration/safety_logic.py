# flake8: noqa
# noqa: D100,D101,D102,D103,D104,D105,D107
"""Pure safety-gate policy; it never sends vehicle commands."""
from dataclasses import dataclass, field


@dataclass
class SafetyInputs:  # noqa: D101
    mode: str = "observe"
    operator_enabled: bool = False
    operator_fresh: bool = False
    px4_fresh: bool = False
    position_fresh: bool = False
    attitude_fresh: bool = False
    failsafe: bool = False
    armed: bool = False
    mission_running: bool = False
    subsystem: dict[str, str] = field(default_factory=dict)
    competition_configured: bool = False


def evaluate_safety(data: SafetyInputs) -> tuple[bool, str]:  # noqa: D103
    if data.mode == "observe":
        return False, "OBSERVE_MODE"
    if data.mode not in {"bench", "hover_test", "drop", "dynamic_land"}:
        return False, "UNKNOWN_MODE"
    if not data.operator_enabled or not data.operator_fresh:
        return False, "OPERATOR_ENABLE_MISSING_OR_STALE"
    if not (data.px4_fresh and data.position_fresh and data.attitude_fresh):
        return False, "PX4_DATA_STALE"
    if data.failsafe:
        return False, "PX4_FAILSAFE"
    if data.armed and not data.mission_running:
        return False, "ARMED_OUTSIDE_MISSION"
    required = {
        "bench": (),
        "hover_test": (),
        "drop": ("vision", "car", "payload"),
        "dynamic_land": ("vision", "car"),
    }[data.mode]
    for name in required:
        if data.subsystem.get(name) not in {"OK", "READY", "ACTIVE"}:
            return False, f"{name.upper()}_NOT_READY"
    if data.mode in {"drop", "dynamic_land"} and not data.competition_configured:
        return False, "HARDWARE_CONFIGURATION_INCOMPLETE"
    return True, "READY"
