#!/usr/bin/env bash
set -euo pipefail
log=${1:?usage: collect_vision_report.sh LOG}; out=${2:-vision_performance.json}
python3 scripts/benchmark/analyze_vision_log.py "$log" --json "$out"
python3 scripts/benchmark/generate_performance_report.py "$out"
