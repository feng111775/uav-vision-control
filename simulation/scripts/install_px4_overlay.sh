#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]] || [[ $# -gt 2 ]]; then
    echo "Usage: $0 /path/to/PX4-Autopilot [--apply]" >&2
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
apply="${2:-}"

echo "PX4 target: $px4_root"
echo "Files to install:"
find "$asset_root" -type f -printf '  %P\n' | sort

if [[ "$apply" != "--apply" ]]; then
    echo "DRY RUN only. Re-run with --apply to copy after review."
    exit 0
fi

version="$(git -C "$px4_root" describe --tags --always 2>/dev/null || true)"
if [[ "$version" != *"v1.17"* ]]; then
    echo "Refusing install: expected PX4 v1.17, found: $version" >&2
    exit 3
fi

backup_root="$px4_root/.qr_shelf_overlay_backup/$(date +%Y%m%d_%H%M%S)"
while IFS= read -r -d '' source; do
    relative="${source#"$asset_root/"}"
    target="$px4_root/$relative"
    if [[ -f "$target" ]]; then
        install -D -m 0644 "$target" "$backup_root/$relative"
    fi
    mode=0644
    [[ "$relative" == ROMFS/* ]] && mode=0755
    install -D -m "$mode" "$source" "$target"
done < <(find "$asset_root" -type f -print0)

cmake_file="$px4_root/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt"
if ! grep -q '4022_gz_x500_downward_camera' "$cmake_file"; then
    indent=$'\t'
    sed -i \
        "/^[[:space:]]*4021_gz_x500_flow/a\\${indent}4022_gz_x500_downward_camera" \
        "$cmake_file"
fi

echo "Installed overlay into: $px4_root"
echo "Replaced files were backed up under: $backup_root"
