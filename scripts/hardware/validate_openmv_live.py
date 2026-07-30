#!/usr/bin/env python3
"""Run OpenMV main.py manually and record protocol/timing observations."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

from openmv_repl import DEFAULT_DEVICE, OpenMVRepl


class Metrics:
    def __init__(self) -> None:
        self.started = time.monotonic()
        self.targets = []
        self.status = Counter()
        self.errors = 0
        self.tracebacks = []
        self.timestamps = []
        self.restarts = 0

    def line(self, text: str) -> None:
        now = time.monotonic()
        if "Traceback" in text or "MemoryError" in text:
            self.tracebacks.append(text)
        if text.startswith("D_STATUS,"):
            self.status[text.split(",", 1)[1].strip()] += 1
        elif text.startswith("D_TARGET,"):
            fields = text.strip().split(",")
            if len(fields) != 8:
                self.errors += 1
                return
            try:
                values = [float(value) for value in fields[1:]]
            except ValueError:
                self.errors += 1
                return
            if not all(math.isfinite(value) for value in values):
                self.errors += 1
                return
            self.targets.append(values)
            self.timestamps.append(now)

    def report(self, phase: str) -> dict:
        intervals = [
            right - left for left, right in zip(self.timestamps, self.timestamps[1:])]
        valid = [row for row in self.targets if int(row[0]) == 1]
        duration = max(time.monotonic() - self.started, 1e-9)
        report = {
            "phase": phase, "duration_s": duration,
            "d_target_count": len(self.targets),
            "valid_0": sum(int(row[0]) == 0 for row in self.targets),
            "valid_1": len(valid), "status": dict(self.status),
            "protocol_errors": self.errors, "tracebacks": self.tracebacks,
            "restart_count": self.restarts,
            "frequency_hz": len(self.targets) / duration,
            "mean_interval_s": statistics.mean(intervals) if intervals else None,
            "p95_interval_s": (
                sorted(intervals)[min(len(intervals) - 1, int(len(intervals) * .95))]
                if intervals else None),
            "max_interval_s": max(intervals) if intervals else None,
        }
        if valid:
            names = ("confidence", "cx", "cy", "outer", "inner", "angle")
            columns = (
                [row[6] for row in valid], [row[1] for row in valid],
                [row[2] for row in valid], [row[3] for row in valid],
                [row[4] for row in valid], [row[5] for row in valid])
            report["valid_metrics"] = {
                name: {"min": min(column), "max": max(column),
                       "mean": statistics.mean(column)}
                for name, column in zip(names, columns)}
            ratios = [row[4] / row[3] for row in valid if row[3] > 0]
            report["inner_outer_ratio"] = {
                "min": min(ratios), "max": max(ratios),
                "mean": statistics.mean(ratios)} if ratios else None
        return report


def capture(repl: OpenMVRepl, duration: float, metrics: Metrics) -> None:
    deadline = time.monotonic() + duration
    pending = ""
    while time.monotonic() < deadline:
        data = repl.serial.read(repl.serial.in_waiting or 1)
        if not data:
            continue
        pending += data.decode("utf-8", errors="replace")
        while "\n" in pending:
            line, pending = pending.split("\n", 1)
            print(line)
            metrics.line(line.strip())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    reports = []
    phases = [("no_target", args.duration)]
    if args.interactive:
        phases = [
            ("no_target", 10), ("center_target", 15), ("move_target", 12),
            ("rotate_target", 12), ("occlude_restore", 10), ("remove_target", 10)]
    with OpenMVRepl(args.device, timeout=5.0) as repl:
        repl.interrupt()
        repl.serial.reset_input_buffer()
        repl.serial.write(b"exec(open('/flash/main.py').read(), {})\r\n")
        repl.serial.flush()
        try:
            for phase, duration in phases:
                if args.interactive:
                    input(f"准备阶段 {phase}（{duration}s），按Enter开始：")
                metrics = Metrics()
                capture(repl, duration, metrics)
                reports.append(metrics.report(phase))
        finally:
            try:
                repl.interrupt()
            except TimeoutError:
                pass
    output = args.output or Path(
        "/tmp/openmv_live_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".json")
    output.write_text(json.dumps(reports, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(output), "reports": reports}, indent=2))
    return 1 if any(report["tracebacks"] or report["protocol_errors"] for report in reports) else 0


if __name__ == "__main__":
    raise SystemExit(main())
