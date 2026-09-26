"""Meshcat adapter for the same RM65 control core used by the hardware entrypoint.

The simulated feedback follows accepted joint commands immediately. It is a
control-logic test, not a model of actuator dynamics or UDP/network latency.
"""

from __future__ import annotations

import copy
import json
import os
import time
import webbrowser
from pathlib import Path

import numpy as np

from xrobotoolkit_teleop.hardware.interface.realman_rm65 import RealmanRM65Interface
from xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller import RM65ControlCore
from xrobotoolkit_teleop.hardware.realman_udp_feedback import ArmFeedback


class SimulatedRM65Arm(RealmanRM65Interface):
    """No socket is opened; accepted movej_follow joints become simulated state."""

    def __init__(self, calibration: dict, initial_joint_rad: np.ndarray, control_period_s: float):
        super().__init__()
        self._calibration = calibration
        self.joint_rad = np.asarray(initial_joint_rad, dtype=float).copy()
        if self.joint_rad.shape != (6,) or not np.all(np.isfinite(self.joint_rad)):
            raise ValueError("simulation home must contain six finite joints")
        self.joint_speed_rad_s = np.zeros(6)
        self._last_command_t = -float("inf")
        self.control_period_s = float(control_period_s)
        self.kinematics = None
        self.last_sent_joint: np.ndarray | None = None
        self.sent_command_count = 0

    def connect(self) -> None:
        pass

    def close(self) -> None:
        pass

    def get_arm_software_info(self) -> dict:
        return copy.deepcopy(self._calibration["arm_info"])

    def send_json(self, payload: dict, wait_s: float) -> list[dict]:
        key = {
            "get_DH_data": "dh",
            "get_current_tool_frame": "tool_frame",
        }.get(payload.get("command"))
        if key is None:
            raise RuntimeError(f"simulation cannot execute {payload.get('command')!r}")
        return [copy.deepcopy(self._calibration[key])]

    def send_movej_follow_joint_rad(self, joint_rad: np.ndarray, active: bool = True) -> bool:
        if not active or self.fault_latched:
            return False
        target = np.asarray(joint_rad, dtype=float)
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            self.latch_fault("simulated movej_follow target is invalid")
            return False
        # Match the real transport's 0.001-degree wire resolution.
        units = self._rad_to_controller_joint(target, self.joint_signs, self.joint_offsets_rad)
        wire_joint = np.deg2rad(np.asarray(units, dtype=float) / 1000.0)
        self.joint_speed_rad_s = (wire_joint - self.joint_rad) / self.control_period_s
        self.joint_rad = wire_joint
        self._last_command_t = time.monotonic()
        self.last_sent_joint = wire_joint.copy()
        self.sent_command_count += 1
        return True

    def slow_stop(self) -> bool:
        self.joint_speed_rad_s = np.zeros(6)
        return True

    def stop(self) -> bool:
        self.joint_speed_rad_s = np.zeros(6)
        return True


class SimulatedRM65Feedback:
    """Create feedback in the configured controller work/TCP frames."""

    thread_error = None
    valid_packet_count = 0
    invalid_packet_count = 0

    def __init__(self, arm: SimulatedRM65Arm):
        self.arm = arm

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def latest(self) -> ArmFeedback | None:
        if self.arm.kinematics is None:
            return None
        now = time.monotonic()
        xyz, quat = self.arm.kinematics.forward(self.arm.joint_rad)
        speed = (
            self.arm.joint_speed_rad_s if now - self.arm._last_command_t < self.arm.control_period_s else np.zeros(6)
        )
        return ArmFeedback(
            received_monotonic_s=now,
            joint_rad=self.arm.joint_rad,
            tcp_xyz_m=xyz,
            tcp_quat_wxyz=quat,
            arm_error_codes=(),
            joint_speed_rad_s=speed,
        )


class RM65SharedControlSimulation(RM65ControlCore):
    """Drive the hardware control state machine with simulated ports and Meshcat."""

    def __init__(
        self,
        robot_urdf_path: str,
        home_joint_deg: tuple[float, float, float, float, float, float],
        calibration_path: str,
        *,
        show_viewer: bool = True,
        **control_kwargs,
    ):
        try:
            calibration = json.loads(Path(calibration_path).read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ValueError(
                "shared-dls needs a controller DH/tool snapshot; run "
                "python scripts/tools/fetch_rm65_dls_calibration.py first"
            ) from exc
        for key in ("arm_info", "dh", "tool_frame"):
            if not isinstance(calibration.get(key), dict):
                raise ValueError(f"calibration snapshot is missing {key}")
        initial_joint = np.deg2rad(np.asarray(home_joint_deg, dtype=float))
        rate_hz = float(control_kwargs.get("control_rate_hz", 50.0))
        arm = SimulatedRM65Arm(calibration, initial_joint, 1.0 / rate_hz)
        feedback = SimulatedRM65Feedback(arm)
        super().__init__(
            arm=arm,
            feedback_receiver=feedback,
            home_joint_deg=home_joint_deg,
            motion_command="damped_ik_movej_follow",
            dry_run=False,
            **control_kwargs,
        )
        arm.kinematics = self.dls.kinematics
        self.simulated_arm = arm
        self.preview_mode = "SIM"

        if show_viewer:
            import placo
            from placo_utils.visualization import robot_viz

            self.placo_robot = placo.RobotWrapper(robot_urdf_path)
            self.placo_robot.state.q[7:] = initial_joint.copy()
            self.placo_robot.update_kinematics()
            self.placo_vis = robot_viz(self.placo_robot)
            if not os.environ.get("MESHCAT_HOST_PORT"):
                webbrowser.open(self.placo_vis.viewer.url())
            self.placo_vis.display(self.placo_robot.state.q)
        else:
            self.placo_robot = None
            self.placo_vis = None

    def _run_cycle(self, now: float) -> None:
        super()._run_cycle(now)
        if self.placo_vis is not None:
            self.placo_robot.state.q[7:] = self.simulated_arm.joint_rad
            self.placo_robot.update_kinematics()
            self.placo_vis.display(self.placo_robot.state.q)
