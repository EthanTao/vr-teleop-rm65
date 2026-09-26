"""Capture read-only RM65 model data for the shared-control simulation backend.

No motion command is sent. Run this in the same container that has the
RealMan API2 SDK, with the controller reachable over TCP.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from xrobotoolkit_teleop.hardware.interface.realman_rm65 import RealmanRM65Interface
from xrobotoolkit_teleop.hardware.realman_official_ik import RealmanOfficialRemoteIK


def _response(arm: RealmanRM65Interface, command: str, field: str, value: str) -> dict:
    for item in reversed(arm.send_json({"command": command}, wait_s=1.0)):
        if item.get(field) == value:
            return item
    raise RuntimeError(f"controller did not return {command}")


def capture(host: str, port: int, output: Path | None) -> None:
    arm = RealmanRM65Interface(host=host, port=port)
    try:
        arm.connect()
        info = arm.get_arm_software_info()
        if not isinstance(info, dict):
            raise RuntimeError("controller software information is unavailable")
        dh = _response(arm, "get_DH_data", "command", "get_DH_data")
        tool = _response(arm, "get_current_tool_frame", "state", "current_tool_frame")
        # Validate the complete snapshot with the exact SDK adapter used by DLS.
        solver = RealmanOfficialRemoteIK(0.02, fk_only=True)
        solver.configure_controller_dh(dh)
        solver.controller_tool_pose(tool)
        data = {
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
            "arm_info": info,
            "dh": dh,
            "tool_frame": tool,
        }
        payload = json.dumps(data, ensure_ascii=False, indent=2) + "\n"
        if output is None:
            print("RM65_CALIBRATION_JSON_BEGIN")
            print(payload, end="")
            print("RM65_CALIBRATION_JSON_END")
        else:
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(payload, encoding="utf-8")
            print(f"[OK] read-only RM65 calibration saved to {output}")
    finally:
        arm.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-host", required=True)
    parser.add_argument("--arm-port", type=int, default=8080)
    parser.add_argument("--output", type=Path, default=Path("config/rm65_dls_calibration.json"))
    args = parser.parse_args()
    capture(args.arm_host, args.arm_port, None if str(args.output) == "-" else args.output)
