#!/usr/bin/env python3
"""Summarize exported JSONL/CSV mission events; missing data remains N/A."""
import argparse
import csv
import json
from collections import Counter


def read_rows(path):
    with open(path, encoding="utf-8") as stream:
        if path.endswith(".jsonl"):
            return [json.loads(line) for line in stream if line.strip()]
        return list(csv.DictReader(stream))


def summarize(rows):
    states = [(row.get("time", "N/A"), row.get("state", "N/A")) for row in rows
              if row.get("state")]
    events = [str(row.get("event", "")) for row in rows]
    valid = [float(row["target_valid"]) for row in rows if row.get("target_valid") not in (None, "")]
    ages = [float(row["target_age_ms"]) for row in rows if row.get("target_age_ms") not in (None, "")]
    return {
        "state_timeline": states,
        "ack_results": Counter(row.get("command_ack", "N/A") for row in rows),
        "takeoff_time": next((t for t, s in states if s == "TAKEOFF"), "N/A"),
        "landing_time": next((t for t, s in states if s == "LAND_H"), "N/A"),
        "b_time": next((row.get("time") for row in rows if str(row.get("car_progress")) == "2"), "N/A"),
        "d_time": next((row.get("time") for row in rows if str(row.get("car_progress")) == "4"), "N/A"),
        "release_count": sum("PAYLOAD_RELEASE" in event for event in events),
        "vision_valid_fraction": sum(valid) / len(valid) if valid else "N/A",
        "mean_target_age_ms": sum(ages) / len(ages) if ages else "N/A",
        "faults": [event for event in events if any(word in event for word in
                   ("FAIL", "ERROR", "ABORT", "EXTERNAL"))],
        "result": states[-1][1] if states else "N/A",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input")
    args = parser.parse_args()
    print(json.dumps(summarize(read_rows(args.input)), indent=2, default=list))


if __name__ == "__main__":
    main()
