#!/usr/bin/env python3
import importlib
import os
import shutil
import sys

EXECUTABLES = (
    "mission_controller_node", "mission_dashboard_node", "car_link_bridge_node",
    "payload_bridge_node", "system_health_node", "safety_gate_node",
)


def main() -> int:
    errors = []
    for module in ("uav_control", "uav_vision", "px4_msgs.msg"):
        try:
            importlib.import_module(module)
        except ImportError as exc:
            errors.append(str(exc))
    for executable in EXECUTABLES:
        if shutil.which(executable) is None:
            errors.append("missing executable: " + executable)
    prefix = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)[0]
    for name in ("d_task_observe.launch.py", "d_task_hardware_bench.launch.py",
                 "d_task_first_flight.launch.py", "d_task_competition.launch.py"):
        if prefix and not os.path.exists(os.path.join(prefix, "share/uav_control/launch", name)):
            errors.append("missing launch: " + name)
    print("\n".join(errors) if errors else "installation verified (no hardware opened)")
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
