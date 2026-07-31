#!/usr/bin/env python3
"""Back up and deploy the four formal OpenMV files without firmware changes."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from openmv_repl import DEFAULT_DEVICE, OpenMVRepl, verify_openmv  # noqa: E402

FORMAL_FILES = (
    "camera_config.py", "detector.py", "detector_fast.py", "protocol.py", "main.py")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            digest.update(block)
    return digest.hexdigest()


def find_mount(device: str) -> tuple[Path, str]:
    verify_openmv(device)
    result = subprocess.run(
        ["findmnt", "-rn", "-S", "LABEL=OPENMV", "-o", "TARGET,SOURCE"],
        check=True, capture_output=True, text=True)
    lines = [line.split() for line in result.stdout.splitlines() if line.strip()]
    if len(lines) != 1:
        raise RuntimeError("expected exactly one mounted OPENMV filesystem")
    mount, source = Path(lines[0][0]).resolve(), lines[0][1]
    if not mount.is_dir() or "OPENMV" not in mount.name.upper():
        raise RuntimeError(f"unsafe OpenMV mount: {mount}")
    return mount, source


def backup(mount: Path, root: Path) -> Path:
    destination = root / ("live_validation_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    files = destination / "files"
    shutil.copytree(mount, files, symlinks=True)
    manifest = []
    for path in sorted(p for p in files.rglob("*") if p.is_file()):
        manifest.append({
            "path": str(path.relative_to(files)), "size": path.stat().st_size,
            "sha256": sha256(path)})
    (destination / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    os.sync()
    if not manifest:
        raise RuntimeError("backup is empty")
    return destination


def copy_verified(source: Path, mount: Path, dry_run: bool) -> dict[str, str]:
    hashes = {}
    for name in FORMAL_FILES:
        origin = source / name
        if not origin.is_file():
            raise FileNotFoundError(origin)
        hashes[name] = sha256(origin)
        if not dry_run:
            shutil.copy2(origin, mount / name)
    if not dry_run:
        os.sync()
        for name, expected in hashes.items():
            if sha256(mount / name) != expected:
                raise RuntimeError(f"SHA256 mismatch after copy: {name}")
    return hashes


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--source", type=Path, default=Path("openmv_h7plus"))
    parser.add_argument(
        "--backup-root", type=Path, default=Path.home() / "openmv_backups")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--rollback", type=Path)
    parser.add_argument("--no-unmount", action="store_true")
    args = parser.parse_args()
    mount, block_device = find_mount(args.device)
    if args.rollback:
        source = args.rollback / "files"
        if not source.is_dir():
            raise RuntimeError("rollback backup has no files directory")
        if not args.dry_run:
            shutil.copytree(source, mount, dirs_exist_ok=True)
            os.sync()
        print(json.dumps({"rollback": str(args.rollback), "dry_run": args.dry_run}))
        return 0
    backup_path = backup(mount, args.backup_root)
    hashes = copy_verified(args.source.resolve(), mount, args.dry_run)
    print(json.dumps({
        "backup": str(backup_path), "mount": str(mount), "hashes": hashes,
        "dry_run": args.dry_run}, indent=2))
    if not args.dry_run and not args.no_unmount:
        subprocess.run(["udisksctl", "unmount", "-b", block_device], check=True)
        with OpenMVRepl(args.device) as repl:
            repl.interrupt()
            print(repl.soft_reset(), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
