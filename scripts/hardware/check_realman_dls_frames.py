#!/usr/bin/env python3
"""Read-only DLS startup and FK/UDP agreement check. No XR or motion commands."""

import argparse
import json
import time

import numpy as np

from xrobotoolkit_teleop.hardware.cartesian_command_limiter import shortest_world_rotation_vector
from xrobotoolkit_teleop.hardware.interface.realman_rm65 import RealmanRM65Interface
from xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller import RealmanRM65SafeTeleopController


class ReadOnlyArm(RealmanRM65Interface):
    def _sendall_observed(self, sock, data, payload):
        allowed = {"get_current_arm_state", "get_arm_software_info", "get_DH_data", "get_current_tool_frame"}
        if payload.get("command") not in allowed:
            raise RuntimeError(f"read-only diagnostic rejected command: {payload}")
        return super()._sendall_observed(sock, data, payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-host", default="192.168.10.18")
    args = parser.parse_args()
    arm = ReadOnlyArm(host=args.arm_host)
    controller = RealmanRM65SafeTeleopController(arm=arm, xr=object(), dry_run=True)
    try:
        controller._startup()
        errors, seen = [], set()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            feedback = controller.feedback_receiver.latest()
            if feedback is not None and feedback.received_monotonic_s not in seen:
                seen.add(feedback.received_monotonic_s)
                controller.dls.begin_cycle()
                controller._dls_frame_matches(feedback)
                xyz, quat = controller.dls.forward(feedback.joint_rad)
                errors.append(
                    [
                        float(np.linalg.norm(xyz - feedback.tcp_xyz_m) * 1000),
                        float(np.rad2deg(np.linalg.norm(shortest_world_rotation_vector(quat, feedback.tcp_quat_wxyz)))),
                    ]
                )
            time.sleep(0.01)
        if len(errors) < 10:
            raise RuntimeError(f"insufficient fresh UDP frames: {len(errors)}")
        print(
            "[PASS] "
            + json.dumps(
                {
                    "frames": len(errors),
                    "max_position_mm": max(e[0] for e in errors),
                    "max_orientation_deg": max(e[1] for e in errors),
                    "motion_commands": 0,
                }
            )
        )
    finally:
        controller.feedback_receiver.stop()
        arm.close()


if __name__ == "__main__":
    main()
