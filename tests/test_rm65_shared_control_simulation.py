"""The Meshcat adapter and hardware entrypoint must run one guarded control core."""

import json

import numpy as np

from tests.test_damped_ik import Q, WristFK, make_dls
from xrobotoolkit_teleop.hardware.cartesian_command_limiter import (
    _angle_axis_quaternion,
    _quaternion_conjugate,
    _quaternion_multiply,
)
import xrobotoolkit_teleop.hardware.interface.realman_rm65 as interface_module
from tests.test_realman_safe_controller import _Xr
from xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller import RM65ControlCore
from xrobotoolkit_teleop.simulation.rm65_shared_control_simulation import RM65SharedControlSimulation


def test_shared_simulation_accepts_same_dls_step_as_hardware(tmp_path, monkeypatch):
    monkeypatch.setattr(
        interface_module.tf,
        "quaternion_about_axis",
        lambda angle, axis: _angle_axis_quaternion(np.asarray(axis) * angle),
        raising=False,
    )
    monkeypatch.setattr(interface_module.tf, "quaternion_inverse", _quaternion_conjugate, raising=False)
    monkeypatch.setattr(interface_module.tf, "quaternion_multiply", _quaternion_multiply, raising=False)
    snapshot = tmp_path / "calibration.json"
    snapshot.write_text(
        json.dumps(
            {
                "arm_info": {"Product_version": "RM65-BI", "robot_controller_version": "3"},
                "dh": {},
                "tool_frame": {},
            }
        ),
        encoding="utf-8",
    )
    sim = RM65SharedControlSimulation(
        robot_urdf_path="unused-when-viewer-disabled",
        home_joint_deg=tuple(np.rad2deg(Q)),
        calibration_path=str(snapshot),
        show_viewer=False,
        xr=_Xr(),
        dls_kinematics=WristFK(),
    )
    assert isinstance(sim, RM65ControlCore)
    sim._startup()
    sim_feedback = sim.feedback_receiver.latest()
    sim.limiter.reset(sim_feedback.tcp_xyz_m, sim_feedback.tcp_quat_wxyz)

    hardware, hardware_arm, hardware_rx = make_dls()
    target_joint = Q.copy()
    target_joint[0] += 0.002
    target_xyz, target_quat = hardware.dls.forward(target_joint)

    assert hardware._send_motion(target_xyz, target_quat, hardware_rx.feedback.joint_rad)
    assert sim._send_motion(target_xyz, target_quat, sim_feedback.joint_rad)
    np.testing.assert_allclose(
        sim.simulated_arm.last_sent_joint,
        hardware_arm.movej_follow_commands[-1],
        atol=1e-12,
    )
    np.testing.assert_allclose(
        sim.feedback_receiver.latest().joint_rad,
        sim.simulated_arm.last_sent_joint,
    )


def test_shared_simulation_rejects_frame_mismatch_without_sending(tmp_path):
    snapshot = tmp_path / "calibration.json"
    snapshot.write_text(
        json.dumps(
            {
                "arm_info": {"Product_version": "RM65-BI", "robot_controller_version": "3"},
                "dh": {},
                "tool_frame": {},
            }
        ),
        encoding="utf-8",
    )
    sim = RM65SharedControlSimulation(
        robot_urdf_path="unused-when-viewer-disabled",
        home_joint_deg=tuple(np.rad2deg(Q)),
        calibration_path=str(snapshot),
        show_viewer=False,
        xr=_Xr(),
        dls_kinematics=WristFK(),
    )
    sim._startup()
    original_latest = sim.feedback_receiver.latest

    def mismatched():
        from dataclasses import replace

        frame = original_latest()
        return replace(frame, tcp_xyz_m=frame.tcp_xyz_m + np.array([0.02, 0, 0]))

    sim.feedback_receiver.latest = mismatched
    q = Q.copy()
    q[0] += 0.002
    xyz, quat = sim.dls.forward(q)
    assert not sim._send_motion(xyz, quat, Q)
    assert sim.simulated_arm.sent_command_count == 0
    assert sim.simulated_arm.fault_latched
