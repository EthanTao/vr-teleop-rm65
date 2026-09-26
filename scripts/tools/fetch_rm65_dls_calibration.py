"""Fetch read-only RM65 DH/tool data into the local simulation snapshot.

This sends the capture script over SSH stdin; it does not install code remotely
or send a motion command.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shlex
import subprocess


def fetch(
    ssh_host: str,
    container: str,
    arm_host: str,
    arm_port: int,
    output: Path,
    python_path: str,
) -> None:
    source = (Path(__file__).resolve().parents[1] / "simulation" / "capture_rm65_calibration.py").read_bytes()
    remote = " ".join(
        shlex.quote(part)
        for part in (
            "sudo",
            "docker",
            "exec",
            "-i",
            "-w",
            "/workspace",
            container,
            python_path,
            "-",
            "--arm-host",
            arm_host,
            "--arm-port",
            str(arm_port),
            "--output",
            "-",
        )
    )
    result = subprocess.run(
        ["ssh", ssh_host, remote],
        input=source,
        capture_output=True,
        check=True,
        timeout=30,
    )
    output_text = result.stdout.decode("utf-8")
    begin = "RM65_CALIBRATION_JSON_BEGIN\n"
    end = "\nRM65_CALIBRATION_JSON_END"
    if begin not in output_text or end not in output_text:
        raise RuntimeError("remote calibration response was incomplete")
    payload = output_text.split(begin, 1)[1].split(end, 1)[0]
    snapshot = json.loads(payload)
    for key in ("arm_info", "dh", "tool_frame"):
        if not isinstance(snapshot.get(key), dict):
            raise ValueError(f"remote snapshot is missing {key}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(snapshot, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"[OK] read-only RM65 calibration saved locally to {output}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ssh-host", default="rm")
    parser.add_argument("--container", default="xrobo-vr-teleop")
    parser.add_argument("--arm-host", default="192.168.10.18")
    parser.add_argument("--arm-port", type=int, default=8080)
    parser.add_argument("--python", default="/opt/conda/bin/python")
    parser.add_argument("--output", type=Path, default=Path("config/rm65_dls_calibration.json"))
    args = parser.parse_args()
    fetch(args.ssh_host, args.container, args.arm_host, args.arm_port, args.output, args.python)
