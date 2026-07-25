#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "Usage: $0 /absolute/path/to/PX4-Autopilot" >&2
    exit 2
fi

px4_root="$1"
if [[ ! -d "$px4_root/ROMFS/px4fmu_common/init.d-posix/airframes" ]] ||
   [[ ! -d "$px4_root/Tools/simulation/gz/models" ]]; then
    echo "Not a PX4-Autopilot tree: $px4_root" >&2
    exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
asset_root="$script_dir/../px4_overlay"

echo "PX4 target: $px4_root"
echo "Files to install:"
find "$asset_root" -type f -printf '  %P\n' | sort

install -D -m 0644 \
    "$asset_root/Tools/simulation/gz/models/x500_downward_camera/model.config" \
    "$px4_root/Tools/simulation/gz/models/x500_downward_camera/model.config"
install -D -m 0644 \
    "$asset_root/Tools/simulation/gz/models/x500_downward_camera/model.sdf" \
    "$px4_root/Tools/simulation/gz/models/x500_downward_camera/model.sdf"
install -D -m 0644 \
    "$asset_root/Tools/simulation/gz/worlds/red_target.sdf" \
    "$px4_root/Tools/simulation/gz/worlds/red_target.sdf"
install -D -m 0755 \
    "$asset_root/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_downward_camera" \
    "$px4_root/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_downward_camera"

cmake_file="$px4_root/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
if ! grep -q '4022_gz_x500_downward_camera' "$cmake_file"; then
    indent=$'\t'
    sed -i \
        "/^[[:space:]]*4021_gz_x500_flow/a\\${indent}4022_gz_x500_downward_camera" \
        "$cmake_file"
fi

echo "Installed verified dual-camera SITL assets into: $px4_root"
