#!/usr/bin/env python3
"""Small, non-destructive OpenMV friendly-REPL client."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from device_identity import verify  # noqa: E402

DEFAULT_DEVICE = "/dev/dtask_openmv"
OPENMV_VID = "37c5"
OPENMV_PID = "124a"
OPENMV_SERIAL = "398239713230"


def verify_openmv(device: str) -> None:
    verify(device, OPENMV_VID, OPENMV_PID, OPENMV_SERIAL)


class OpenMVRepl:
    def __init__(self, device: str = DEFAULT_DEVICE, timeout: float = 3.0):
        verify_openmv(device)
        self.serial = serial.Serial(device, 115200, timeout=0.05, write_timeout=timeout)
        self.timeout = timeout

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def __enter__(self) -> "OpenMVRepl":
        return self

    def __exit__(self, *_args) -> None:
        self.close()

    def _read_until(self, marker: bytes, timeout: float | None = None) -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        output = bytearray()
        while time.monotonic() < deadline:
            output.extend(self.serial.read(self.serial.in_waiting or 1))
            if marker in output:
                return bytes(output)
        raise TimeoutError(
            f"timeout waiting for {marker!r}; output="
            f"{bytes(output).decode('utf-8', errors='replace')!r}")

    def interrupt(self) -> str:
        self.serial.reset_input_buffer()
        self.serial.write(b"\x03")
        self.serial.flush()
        output = self._read_until(b">>> ", max(self.timeout, 5.0))
        # A stopped OpenMV script may finish writing one buffered line after
        # the prompt. Let the CDC endpoint settle so it cannot satisfy the
        # next command's prompt wait.
        time.sleep(0.1)
        output += self.serial.read(self.serial.in_waiting)
        return output.decode("utf-8", errors="replace")

    def execute(self, expression: str, timeout: float | None = None) -> str:
        self.serial.reset_input_buffer()
        command = ("exec(" + repr(expression) + ")\r\n").encode("utf-8")
        self.serial.write(command)
        self.serial.flush()
        raw = self._read_until(b">>> ", timeout)
        return raw.decode("utf-8", errors="replace")

    def soft_reset(self, timeout: float = 8.0) -> str:
        self.serial.reset_input_buffer()
        self.serial.write(b"\x04")
        self.serial.flush()
        return self._read_until(b">>> ", timeout).decode("utf-8", errors="replace")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--timeout", type=float, default=3.0)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--exec", dest="expression")
    group.add_argument("--soft-reset", action="store_true")
    group.add_argument("--interrupt", action="store_true")
    args = parser.parse_args()
    with OpenMVRepl(args.device, args.timeout) as repl:
        if args.interrupt:
            output = repl.interrupt()
        elif args.soft_reset:
            repl.interrupt()
            output = repl.soft_reset()
        else:
            repl.interrupt()
            output = repl.execute(args.expression, args.timeout)
    print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
