"""Safe udev identity lookup shared by command-line tools."""
import os
import subprocess


def properties(device: str) -> dict[str, str]:
    if not os.path.exists(device):
        raise ValueError(f"device does not exist: {device}")
    result = subprocess.run(
        ["udevadm", "info", "--query=property", f"--name={device}"],
        check=True, capture_output=True, text=True,
    )
    return dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)


def verify(device: str, vid: str, pid: str, serial: str) -> dict[str, str]:
    if not (vid and pid and serial):
        raise ValueError("expected VID, PID, and serial must all be explicit")
    found = properties(device)
    expected = {
        "ID_VENDOR_ID": vid.lower(), "ID_MODEL_ID": pid.lower(),
        "ID_SERIAL_SHORT": serial,
    }
    for key, value in expected.items():
        if found.get(key, "").lower() != value.lower():
            raise ValueError(f"{key} mismatch")
    return found
