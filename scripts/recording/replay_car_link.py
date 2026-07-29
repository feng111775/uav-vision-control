#!/usr/bin/env python3
"""Replay validated JSONL car frames to localhost UDP for bench tests."""
import argparse
import json
import socket
import time

from uav_control.hardware.car_link_protocol import CarFrame, encode_car_frame


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, required=True)
    args = parser.parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.monotonic()
    with open(args.input, encoding="utf-8") as stream:
        for line in stream:
            item = json.loads(line)
            delay = float(item["at_seconds"]) - (time.monotonic() - start)
            if delay > 0:
                time.sleep(delay)
            frame = CarFrame(item["sequence"], item["timestamp_ms"],
                             item["progress"], item["flags"])
            sock.sendto(encode_car_frame(frame).encode(), (args.host, args.port))


if __name__ == "__main__":
    main()
