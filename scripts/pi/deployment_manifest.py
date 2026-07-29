"""Create a deployable list using Git, never build products or datasets."""
import hashlib
import os
import subprocess
import sys

EXCLUDED = ("build", "install", "log", "datasets/", "models/", ".git/")


def tracked_manifest(root: str) -> list[tuple[str, str]]:
    output = subprocess.run(
        ["git", "-C", root, "ls-files"], check=True,
        capture_output=True, text=True,
    ).stdout
    result = []
    for relative in output.splitlines():
        if any(relative == item or relative.startswith(item) for item in EXCLUDED):
            continue
        path = os.path.join(root, relative)
        if os.path.isfile(path):
            with open(path, "rb") as stream:
                digest = hashlib.sha256(stream.read()).hexdigest()
            result.append((relative, digest))
    return result


if __name__ == '__main__':
    for name, digest in tracked_manifest(sys.argv[1]):
        print(f'{digest}  {name}')
