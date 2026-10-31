"""Restart only after a fresh, explicit request from this owned sim session.

No robot commands, Docker cleanup, world resets, or retries after ordinary errors.
"""

from __future__ import annotations

import json
from pathlib import Path
import re
import secrets
import subprocess
import time
from typing import Callable


def consume_request(path: Path, token: str, started: float) -> bool:
    if not re.fullmatch(r"[0-9a-f]{32}", token):
        raise ValueError("Invalid simulation session token")
    if path.is_symlink() or path.parent.is_symlink():
        raise ValueError("Invalid simulation reset request file")
    if not path.exists():
        return False
    if not path.is_file() or path.stat().st_size > 2048:
        raise ValueError("Invalid simulation reset request file")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict) or set(value) != {"version", "mode", "action", "token", "requested_at"}:
        raise ValueError("Invalid simulation reset request shape")
    stamp = value["requested_at"]
    if (
        type(value["version"]) is not int
        or value["version"] != 1
        or value["mode"] != "sim"
        or value["action"] != "reset"
        or value["token"] != token
        or isinstance(stamp, bool)
        or not isinstance(stamp, (int, float))
        or not started <= stamp <= time.time() + 0.1
    ):
        raise ValueError("Stale or unowned simulation reset request")
    path.unlink()
    return True


def run_simulation(command: list[str], *, root: Path, execute: Callable, announce: Callable) -> None:
    """Each explicit reset gets a new token and a completely new ROS graph."""
    folder = root / "build/experiments/.sim-reset"
    folder.mkdir(parents=True, exist_ok=True)
    while True:
        token = secrets.token_hex(16)
        request = folder / f"{token}.json"
        started = time.time()
        error = None
        try:
            execute([*command, "--sim-reset-token", token])
        except subprocess.CalledProcessError as exc:
            error = exc
        # KeyboardInterrupt/OSError/other failures deliberately never reach here.
        if not consume_request(request, token, started):
            if error is not None:
                raise error
            return
        announce("[复位] 已结束旧仿真会话；重新建立初始姿态，清空旧目标。头显重连后须先松 Grip。")
