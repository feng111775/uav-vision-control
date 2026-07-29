#!/usr/bin/env python3
"""Report deployment prerequisites without changing the machine."""
import importlib.util
import json
import os
import platform
import shutil
import subprocess


def command(args):
    try:
        return subprocess.run(args, capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def main() -> int:
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
    report = {
        "architecture": platform.machine(),
        "ubuntu": command(["lsb_release", "-ds"]),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "python": platform.python_version(),
        "colcon": shutil.which("colcon"),
        "pyserial": importlib.util.find_spec("serial") is not None,
        "opencv": importlib.util.find_spec("cv2") is not None,
        "numpy": importlib.util.find_spec("numpy") is not None,
        "agent": shutil.which("MicroXRCEAgent"),
        "git_head": command(["git", "-C", root, "rev-parse", "HEAD"]),
        "px4_msgs_head": command(["git", "-C", root + "/src/px4_msgs", "rev-parse", "HEAD"]),
        "disk_free_bytes": shutil.disk_usage(root).free,
        "network_interfaces": sorted(os.listdir("/sys/class/net")),
    }
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
