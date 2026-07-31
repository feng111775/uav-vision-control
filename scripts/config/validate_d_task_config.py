#!/usr/bin/env python3
"""Reject unsafe D-task hardware/mission configurations."""
import argparse
import os
import re
import sys
import yaml

LOCAL_POSITION = "/fmu/out/vehicle_local_position"
VEHICLE_STATUS = "/fmu/out/vehicle_status_v1"


def flatten(value, prefix=""):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from flatten(child, f"{prefix}.{key}" if prefix else key)
    else:
        yield prefix, value


def validate_data(data, name, source_text=""):
    flat = dict(flatten(data))
    values = list(flat.values())
    errors = []
    mode = next((v for k, v in flat.items() if k.endswith("mission_mode")), None)
    if mode is not None and mode not in {"drop", "dynamic_land", "hover_test"}:
        errors.append("invalid mission_mode")
    competition = "competition" in os.path.basename(name)
    basename = os.path.basename(name)
    first = basename in {"first_flight_real.yaml", "first_flight_bench.yaml"}
    if competition:
        if any(str(v).lower() == "mock" for v in values):
            errors.append("mock transport in competition")
        for suffix in ("enable_control", "enable_auto_arm"):
            if any(k.endswith(suffix) and v is not False for k, v in flat.items()):
                errors.append(suffix + " must default false")
    if first:
        for suffix in ("enable_visual_follow", "enable_payload_release",
                       "enable_dynamic_landing", "enable_second_takeoff",
                       "enable_auto_arm"):
            if any(k.endswith(suffix) and bool(v) for k, v in flat.items()):
                errors.append("unsafe first-flight option: " + suffix)
        if any(k.endswith("enable_control") and v is not True for k, v in flat.items()):
            errors.append("first-flight launch must keep enable_control true")
        if mode is not None and mode != "hover_test":
            errors.append("first-flight launch must use hover_test mission_mode")
    for key, value in flat.items():
        if key.endswith("transport") and value == "serial":
            base = key.rsplit(".", 1)[0]
            port = flat.get(base + ".device", flat.get(base + ".serial_port"))
            baud = flat.get(base + ".baudrate")
            if not port or not baud:
                errors.append("serial transport missing device/baudrate")
        if key.endswith("transport") and value == "udp":
            base = key.rsplit(".", 1)[0]
            if not flat.get(base + ".bind_port"):
                errors.append("udp transport missing bind_port")
    text = source_text
    if "/dev/ttyACM0" in text and "openmv" in text.lower():
        errors.append("OpenMV must not be hard-coded as /dev/ttyACM0")
    if re.search(r"\bgpio(?:_pin)?\s*:\s*\d+", text, re.I):
        errors.append("unverified GPIO pin")
    return errors


def validate_file(path):
    with open(path, encoding="utf-8") as stream:
        text = stream.read()
    return validate_data(yaml.safe_load(text) or {}, str(path), text)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="+")
    args = parser.parse_args()
    errors = [(path, error) for path in args.paths for error in validate_file(path)]
    for path, error in errors:
        print(f"{path}: {error}", file=sys.stderr)
    return bool(errors)


if __name__ == "__main__":
    raise SystemExit(main())
