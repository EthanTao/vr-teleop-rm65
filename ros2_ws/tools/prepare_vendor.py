#!/usr/bin/env python3
"""Apply the pinned driver's transport patch and choose its native SDK library."""

from __future__ import annotations

import argparse
from pathlib import Path
import platform
import shutil
import subprocess

EXPECTED_COMMIT = "61db3c367e8f5743ee68cee52c5574ec1ac645ca"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    vendor = workspace / "src/vendor/rm_robot"
    commit = subprocess.check_output(["git", "-C", str(vendor), "rev-parse", "HEAD"], text=True).strip()
    if commit != EXPECTED_COMMIT:
        raise RuntimeError(f"driver commit mismatch: {commit}")
    patch = workspace / "patches/realman_driver_feedback.patch"
    if not patch.is_file():
        raise RuntimeError("driver feedback/stop patch is required; refusing an unpatched hardware driver")
    reverse = subprocess.run(
        ["git", "-C", str(vendor), "apply", "--reverse", "--check", str(patch)], capture_output=True
    )
    if reverse.returncode != 0:
        subprocess.run(["git", "-C", str(vendor), "apply", "--check", str(patch)], check=True)
        subprocess.run(["git", "-C", str(vendor), "apply", str(patch)], check=True)
    architecture = platform.machine().lower()
    native = {
        "aarch64": "linux_arm64_c++_v1.1.5",
        "arm64": "linux_arm64_c++_v1.1.5",
        "x86_64": "linux_x86_c++_v1.1.5",
        "amd64": "linux_x86_c++_v1.1.5",
    }.get(architecture)
    if native is None:
        raise RuntimeError(f"unsupported vendor SDK architecture: {architecture}")
    source = vendor / "rm_driver/lib" / native / "libapi_cpp.so"
    if not source.is_file():
        raise RuntimeError(f"missing native vendor SDK: {source}")
    shutil.copyfile(source, vendor / "rm_driver/lib/libapi_cpp.so")
    print(f"Prepared official driver {commit}; native SDK {native}; no SDK execution or arm connection")


if __name__ == "__main__":
    main()
