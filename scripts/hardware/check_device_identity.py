#!/usr/bin/env python3
import argparse
import json
from device_identity import verify


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("device")
    parser.add_argument("--vid", required=True)
    parser.add_argument("--pid", required=True)
    parser.add_argument("--serial", required=True)
    args = parser.parse_args()
    print(json.dumps(verify(args.device, args.vid, args.pid, args.serial), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
