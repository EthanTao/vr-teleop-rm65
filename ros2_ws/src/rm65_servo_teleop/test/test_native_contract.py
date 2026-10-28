"""Compile the actual added vendor header against fake ROS/SDK interfaces.

This checks the native gate logic; it is not a ROS ABI or physical-arm test.
"""

from pathlib import Path
import shutil
import subprocess
import sys

import pytest


def added_contract(patch: Path) -> str:
    in_header = False
    result = []
    for line in patch.read_text(encoding="utf-8").splitlines(keepends=True):
        if line.startswith("+++ "):
            in_header = line.rstrip().endswith("/rm65_servo_contract.h")
        elif in_header and line.startswith("+"):
            result.append(line[1:])
    return "".join(result)


def test_native_generation_stop_and_watchdog(tmp_path):
    compiler = shutil.which("g++")
    if compiler is None:
        pytest.skip("native mock requires a C++14 compiler; full ROS build remains separate")
    test_dir = Path(__file__).parent
    patch = test_dir.parents[2] / "patches" / "realman_driver_feedback.patch"
    source = added_contract(patch)
    assert source.startswith("// Copyright")
    boot_file = tmp_path / "mock_boot_id"
    boot_file.write_text("mock-linux-boot\n", encoding="ascii")
    # Only the OS-specific source for the boot ID is injected; the gate,
    # generation, timestamp, callback and mutex logic is the actual vendor patch.
    source = source.replace("/proc/sys/kernel/random/boot_id", boot_file.as_posix())
    (tmp_path / "native_contract_under_test.h").write_text(source, encoding="utf-8")
    binary = tmp_path / ("native_contract_mock.exe" if sys.platform == "win32" else "native_contract_mock")
    build = subprocess.run(
        [
            compiler,
            "-std=c++14",
            "-pthread",
            "-I",
            str(test_dir / "native_stubs"),
            "-I",
            str(tmp_path),
            str(test_dir / "native_contract_mock.cpp"),
            "-o",
            str(binary),
        ],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(binary)], capture_output=True, text=True, timeout=10, check=False)
    assert run.returncode == 0, run.stderr
    assert "serialized stop, watchdog, failed-stop lock passed" in run.stdout
