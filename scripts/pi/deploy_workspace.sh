#!/usr/bin/env bash
set -euo pipefail
apply=false
target=""
while (($#)); do
  case "$1" in
    --apply) apply=true; shift;;
    --target) target="$2"; shift 2;;
    *) echo "unknown argument: $1" >&2; exit 2;;
  esac
done
[ -n "$target" ] || { echo "--target required" >&2; exit 2; }
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
checksum_manifest="$(mktemp /tmp/d-task-sha256.XXXXXX)"
manifest="$(mktemp /tmp/d-task-files.XXXXXX)"
python3 "$root/scripts/pi/deployment_manifest.py" "$root" >"$checksum_manifest"
sed 's/^[0-9a-f]*  //' "$checksum_manifest" >"$manifest"
echo "dry-run=$([ "$apply" = true ] && echo false || echo true) files=$(wc -l <"$manifest") target=$target"
echo "SHA256 manifest: $checksum_manifest"
if ! $apply; then exit 0; fi
if [[ "$target" == *:* ]]; then
  host="${target%%:*}"; path="${target#*:}"
  dirty="$(ssh "$host" "test ! -d '$path/.git' || git -C '$path' status --porcelain")"
  [ -z "$dirty" ] || { echo "target has uncommitted changes" >&2; exit 1; }
  rsync -a --files-from="$manifest" "$root/" "$target/"
else
  if [ -d "$target/.git" ] && [ -n "$(git -C "$target" status --porcelain)" ]; then
    echo "target has uncommitted changes" >&2; exit 1
  fi
  mkdir -p "$target"
  rsync -a --files-from="$manifest" "$root/" "$target/"
fi
