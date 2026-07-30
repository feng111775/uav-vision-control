#!/usr/bin/env bash
set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
result_dir="${1:-${PWD}/test_results/sitl_hover_regression}"
"${script_dir}/run_sitl_hover.sh" nominal 3 "${result_dir}"
