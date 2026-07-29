#!/usr/bin/env python3
import argparse
from device_identity import properties

ALIASES = {"dtask_pixhawk", "dtask_openmv", "dtask_car", "dtask_payload"}


def build_rule(device: str, alias: str) -> str:
    if alias not in ALIASES:
        raise ValueError("alias is not in the approved D-task list")
    props = properties(device)
    vid = props.get("ID_VENDOR_ID")
    pid = props.get("ID_MODEL_ID")
    serial = props.get("ID_SERIAL_SHORT")
    if not (vid and pid and serial):
        raise ValueError("VID/PID/serial are insufficient for a unique rule")
    return (
        'SUBSYSTEM=="tty", ATTRS{idVendor}=="%s", ATTRS{idProduct}=="%s", '
        'ATTRS{serial}=="%s", SYMLINK+="%s"\n' % (vid, pid, serial, alias)
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("device")
    parser.add_argument("alias", choices=sorted(ALIASES))
    parser.add_argument("--output")
    args = parser.parse_args()
    rule = build_rule(args.device, args.alias)
    if args.output:
        with open(args.output, "x", encoding="utf-8") as stream:
            stream.write(rule)
    else:
        print(rule, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
