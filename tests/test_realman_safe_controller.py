import sys
import types
from dataclasses import replace
import math

import numpy as np
import pytest

if "meshcat.transformations" not in sys.modules:
    meshcat = types.ModuleType("meshcat")
    transformations = types.ModuleType("meshcat.transformations")
    meshcat.transformations = transformations
    sys.modules.setdefault("meshcat", meshcat)
    sys.modules.setdefault("meshcat.transformations", transformations)

if "xrobotoolkit_sdk" not in sys.modules:
    sys.modules["xrobotoolkit_sdk"] = types.ModuleType("xrobotoolkit_sdk")

from xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller import (
    RealmanRM65SafeTeleopController,
)
import xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller as safe_controller_module
from xrobotoolkit_teleop.hardware.realman_udp_feedback import ArmFeedback
from xrobotoolkit_teleop.hardware.singularity_avoidance import rm65_geometric_jacobian


class _Arm:
    def __init__(self):
        self.fault_latched = False
        self.fault_reason = None
        self.latch_calls = 0
        self.clear_calls = 0
        self.slow_stop_calls = 0
        self.movep_follow_commands = []
        self.movej_follow_commands = []
        self.command_reference = None

    def latch_fault(self, reason):
        self.fault_latched = True
        self.fault_reason = reason
        self.latch_calls += 1

    def clear_fault(self):
        self.fault_latched = False
        self.fault_reason = None
        self.clear_calls += 1

    def slow_stop(self):
        self.slow_stop_calls += 1
        return True

    def get_last_commanded_pose6(self):
        return None

    def send_movep_follow_xyz_quat(self, xyz, quat, active=True):
        self.movep_follow_commands.append((np.asarray(xyz).copy(), np.asarray(quat).copy()))
        return True

    def send_movej_follow_joint_rad(self, joint_rad, active=True):
        self.movej_follow_commands.append(np.asarray(joint_rad).copy())
        return True

    def xyz_quat_to_pose6(self, xyz, quat):
        return [1, 2, 3, 4, 5, 6]

    def sync_command_reference(self, pose6):
        self.command_reference = list(pose6)


class _Xr:
    def __init__(self, grip=0.0):
        self.grip = grip
        self.timestamp_ns = 1
        self.pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    def get_key_value_by_name(self, name):
        return self.grip

    def get_button_state_by_name(self, name):
        return False

    def get_timestamp_ns(self):
        return self.timestamp_ns

    def get_pose_by_name(self, name):
        return self.pose.copy()


class _FeedbackReceiver:
    def __init__(self, feedback):
        self.feedback = feedback
        self.thread_error = None

    def latest(self):
        return self.feedback


class _IKSolver:
    def __init__(self, result=None, error=None):
        self.result = np.zeros(6) if result is None else np.asarray(result, dtype=float)
        self.error = error
        self.calls = []

    def solve(self, xyz, quat, seed_joint_rad):
        self.calls.append(
            (
                np.asarray(xyz).copy(),
                np.asarray(quat).copy(),
                np.asarray(seed_joint_rad).copy(),
            )
        )
        if self.error is not None:
            raise self.error
        return self.result.copy()


def _feedback(timestamp=1.0, joint_speed_rad_s=None):
    if joint_speed_rad_s is None:
        joint_speed_rad_s = np.zeros(6)
    return ArmFeedback(
        received_monotonic_s=timestamp,
        joint_rad=np.zeros(6),
        tcp_xyz_m=np.array([0.3, 0.0, 0.5]),
        tcp_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        arm_error_codes=(),
        joint_speed_rad_s=np.asarray(joint_speed_rad_s, dtype=float),
    )


def _controller(feedback, **kwargs):
    # Historical path tests explicitly select the legacy mode.
    kwargs.setdefault("motion_command", "movep_follow")
    kwargs.setdefault("enable_singularity_avoidance", False)
    arm = _Arm()
    receiver = _FeedbackReceiver(feedback)
    controller = RealmanRM65SafeTeleopController(
        arm=arm,
        xr=_Xr(),
        feedback_receiver=receiver,
        **kwargs,
    )
    return controller, arm, receiver


def test_joint_overspeed_latches_fault_and_requires_button_rearm():
    speed = np.zeros(6)
    speed[0] = np.deg2rad(19.0)
    controller, arm, _ = _controller(_feedback(joint_speed_rad_s=speed))

    result = controller._checked_feedback(1.0, require_fresh=True)

    assert result is None
    assert arm.fault_latched
    assert "J1 overspeed" in arm.fault_reason
    assert controller._rearm_required
    assert controller._rearm_button_required


def test_stale_udp_feedback_latches_fault_fail_closed():
    controller, arm, _ = _controller(_feedback(timestamp=1.0))

    result = controller._checked_feedback(1.21, require_fresh=True)

    assert result is None
    assert arm.fault_latched
    assert "feedback stale" in arm.fault_reason


def test_fault_rearm_requires_release_then_button_and_stationary_feedback():
    controller, arm, receiver = _controller(_feedback())
    controller._latch_safety_fault("test fault")

    controller._handle_rearm(grip=1.0, reset_edge=True, now=1.0)
    assert arm.clear_calls == 0

    controller._handle_rearm(grip=0.0, reset_edge=False, now=1.0)
    moving = np.zeros(6)
    moving[0] = np.deg2rad(2.0)
    receiver.feedback = _feedback(joint_speed_rad_s=moving)
    controller._handle_rearm(grip=0.0, reset_edge=True, now=1.0)
    assert arm.clear_calls == 0

    receiver.feedback = _feedback(joint_speed_rad_s=np.zeros(6))
    controller._handle_rearm(grip=0.0, reset_edge=True, now=1.0)

    assert arm.clear_calls == 1
    assert not controller._rearm_required


def test_rotation_scale_factor_reduces_xr_rotation(monkeypatch):
    controller, _, _ = _controller(
        _feedback(),
        rotation_scale_factor=0.4,
        rot_deadband_rad=0.0,
        use_headset_world_transform=False,
    )
    controller.ref_ctrl_xyz = np.zeros(3)
    controller.ref_ctrl_quat = np.array([1.0, 0.0, 0.0, 0.0])
    monkeypatch.setattr(
        safe_controller_module.tf,
        "quaternion_from_matrix",
        lambda matrix: np.array([1.0, 0.0, 0.0, 0.0]),
        raising=False,
    )
    monkeypatch.setattr(safe_controller_module.tf, "quaternion_multiply", lambda left, right: right, raising=False)
    monkeypatch.setattr(safe_controller_module.tf, "quaternion_conjugate", lambda quat: quat, raising=False)
    monkeypatch.setattr(
        safe_controller_module,
        "quat_diff_as_angle_axis",
        lambda source, target: np.array([0.5, 0.0, 0.0]),
    )

    _, delta_rot = controller._process_xr_pose(controller.xr.pose)
    np.testing.assert_allclose(delta_rot, [0.2, 0.0, 0.0])


def test_grip_release_uses_slow_stop_instead_of_hard_stop():
    controller, arm, _ = _controller(_feedback())
    controller.grip_active = True
    controller.xr.grip = 0.0

    controller._run_cycle(1.0)

    assert arm.slow_stop_calls == 1
    assert not controller.grip_active


def test_active_cycle_uses_latest_udp_feedback_and_movep_follow(monkeypatch):
    feedback = _feedback()
    controller, arm, _ = _controller(feedback)
    controller.grip_active = True
    controller.xr.grip = 1.0
    controller.ref_arm_xyz = feedback.tcp_xyz_m.copy()
    controller.ref_arm_quat = feedback.tcp_quat_wxyz.copy()
    controller._last_valid_xr_pose = controller.xr.pose.copy()
    controller._last_valid_xr_t = 0.98
    controller.limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
    monkeypatch.setattr(
        controller,
        "_process_xr_pose",
        lambda pose: (np.zeros(3), np.zeros(3)),
    )
    monkeypatch.setattr(
        safe_controller_module,
        "apply_delta_pose",
        lambda xyz, quat, delta_xyz, delta_rot: (xyz.copy(), quat.copy()),
    )

    controller._run_cycle(1.0)

    assert len(arm.movep_follow_commands) == 1
    np.testing.assert_allclose(arm.movep_follow_commands[0][0], feedback.tcp_xyz_m)
    assert arm.latch_calls == 0


def test_xr_single_frame_jump_hard_stops_and_clears_active_grip():
    feedback = _feedback()
    controller, arm, _ = _controller(feedback)
    controller.grip_active = True
    controller.xr.grip = 1.0
    controller.ref_arm_xyz = feedback.tcp_xyz_m.copy()
    controller.ref_arm_quat = feedback.tcp_quat_wxyz.copy()
    controller._last_valid_xr_pose = controller.xr.pose.copy()
    controller._last_valid_xr_t = 0.98
    controller.xr.pose[0] = 0.051

    controller._run_cycle(1.0)

    assert arm.latch_calls == 1
    assert "XR pose jump" in arm.fault_reason
    assert controller._rearm_required
    assert not controller.grip_active


def test_official_ik_mode_uses_udp_seed_and_movej_follow():
    feedback = _feedback()
    solver = _IKSolver(result=np.deg2rad([0.1, 0.0, 0.0, 0.0, 0.0, 0.0]))
    controller, arm, _ = _controller(
        feedback,
        motion_command="official_ik_movej_follow",
        official_ik_frames_verified=True,
        official_ik_j3_soft_limit_verified=True,
        ik_solver=solver,
    )

    sent = controller._send_motion(
        feedback.tcp_xyz_m,
        feedback.tcp_quat_wxyz,
        feedback.joint_rad,
    )

    assert sent
    assert len(solver.calls) == 1
    np.testing.assert_allclose(solver.calls[0][2], feedback.joint_rad)
    np.testing.assert_allclose(arm.movej_follow_commands[0], solver.result)
    assert arm.command_reference == [1, 2, 3, 4, 5, 6]


def test_official_ik_failure_latches_fault_without_sending_joint_target():
    feedback = _feedback()
    solver = _IKSolver(error=RuntimeError("no solution"))
    controller, arm, _ = _controller(
        feedback,
        motion_command="official_ik_movej_follow",
        official_ik_frames_verified=True,
        official_ik_j3_soft_limit_verified=True,
        ik_solver=solver,
    )

    sent = controller._send_motion(
        feedback.tcp_xyz_m,
        feedback.tcp_quat_wxyz,
        feedback.joint_rad,
    )

    assert not sent
    assert arm.fault_latched
    assert "official RM65 IK failed" in arm.fault_reason
    assert arm.movej_follow_commands == []


def test_official_ik_joint_step_limit_fails_closed():
    feedback = _feedback()
    solver = _IKSolver(result=np.deg2rad([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
    controller, arm, _ = _controller(
        feedback,
        motion_command="official_ik_movej_follow",
        official_ik_frames_verified=True,
        official_ik_j3_soft_limit_verified=True,
        ik_solver=solver,
    )

    sent = controller._send_motion(
        feedback.tcp_xyz_m,
        feedback.tcp_quat_wxyz,
        feedback.joint_rad,
    )

    assert not sent
    assert arm.fault_latched
    assert "J1 step exceeded limit" in arm.fault_reason
    assert arm.movej_follow_commands == []


def test_official_ik_dry_run_solves_but_does_not_send():
    feedback = _feedback()
    solver = _IKSolver(result=np.zeros(6))
    controller, arm, _ = _controller(
        feedback,
        motion_command="official_ik_movej_follow",
        official_ik_frames_verified=True,
        official_ik_j3_soft_limit_verified=True,
        ik_solver=solver,
        dry_run=True,
    )

    assert controller._send_motion(
        feedback.tcp_xyz_m,
        feedback.tcp_quat_wxyz,
        feedback.joint_rad,
    )
    assert len(solver.calls) == 1
    assert arm.movej_follow_commands == []


@pytest.mark.parametrize(
    ("extra", "message"),
    [
        ({"official_ik_j3_soft_limit_verified": True}, "frames_verified"),
        ({"official_ik_frames_verified": True}, "j3_soft_limit_verified"),
    ],
)
def test_official_ik_mode_requires_explicit_safety_verification(extra, message):
    with pytest.raises(ValueError, match=message):
        _controller(
            _feedback(),
            motion_command="official_ik_movej_follow",
            ik_solver=_IKSolver(),
            **extra,
        )


@pytest.mark.parametrize("udp_port", [0, 65536, True])
def test_controller_rejects_invalid_udp_port_before_startup(udp_port):
    with pytest.raises(ValueError, match="udp_port"):
        RealmanRM65SafeTeleopController(
            udp_port=udp_port,
            arm=_Arm(),
            xr=_Xr(),
            feedback_receiver=_FeedbackReceiver(_feedback()),
        )


def test_safe_controller_does_not_allow_release_stop_to_be_disabled():
    with pytest.raises(ValueError, match="cannot be disabled"):
        RealmanRM65SafeTeleopController(
            send_stop_on_release=False,
            arm=_Arm(),
            xr=_Xr(),
            feedback_receiver=_FeedbackReceiver(_feedback()),
        )


# --- singularity avoidance -------------------------------------------------------------------

CLEAR_JOINTS = np.deg2rad([0.0, -30.0, 60.0, 0.0, 60.0, 0.0])
# J5 = 3 deg: inside the documented wrist-singularity stop ramp.
WRIST_DANGER_JOINTS = np.deg2rad([0.0, -30.0, 55.0, 0.0, 3.0, 0.0])


def _joint_feedback(joint_rad, timestamp=1.0, tcp_xyz=np.array([0.3, 0.0, 0.5])):
    return replace(
        _feedback(timestamp=timestamp),
        joint_rad=np.asarray(joint_rad, dtype=float),
        tcp_xyz_m=np.asarray(tcp_xyz, dtype=float),
    )


def _inward_twist(joint_rad):
    """6-D task twist that drives |J5| further down at the given configuration."""
    column = rm65_geometric_jacobian(np.asarray(joint_rad, dtype=float))[:, 4]
    # The fixtures use J5 > 0, so "further in" is the negative of the J5 Jacobian column.
    step = -column if joint_rad[4] > 0.0 else column
    return step * (0.001 / float(np.linalg.norm(step[:3])))


def _rotate_quaternion(quat_wxyz, rotation_vector):
    """Pure-NumPy angle-axis composition so tests do not need meshcat."""
    angle = float(np.linalg.norm(rotation_vector))
    quat = np.asarray(quat_wxyz, dtype=float)
    if angle == 0.0:
        return quat.copy()
    axis = np.asarray(rotation_vector, dtype=float) / angle
    delta = np.concatenate(([math.cos(angle / 2.0)], axis * math.sin(angle / 2.0)))
    w1, v1 = float(delta[0]), delta[1:]
    w2, v2 = float(quat[0]), quat[1:]
    return np.concatenate(([w1 * w2 - float(np.dot(v1, v2))], w1 * v2 + w2 * v1 + np.cross(v1, v2)))


def _guarded_controller(feedback, twist, **kwargs):
    controller, arm, receiver = _controller(feedback, **kwargs)
    controller.grip_active = True
    controller.xr.grip = 1.0
    controller.ref_arm_xyz = feedback.tcp_xyz_m.copy()
    controller.ref_arm_quat = feedback.tcp_quat_wxyz.copy()
    controller._last_valid_xr_pose = controller.xr.pose.copy()
    controller._last_valid_xr_t = 0.98
    controller.limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
    monkeypatch_twist = (twist[:3].copy(), twist[3:].copy())
    return controller, arm, receiver, monkeypatch_twist


def test_clear_pose_keeps_the_command_path_untouched(monkeypatch):
    from tests.test_damped_ik import make_dls, send_step

    controller, arm, receiver = make_dls()
    assert send_step(controller, receiver)
    assert not controller._singularity_hold
    assert len(arm.movej_follow_commands) == 1
    assert arm.movep_follow_commands == []
    assert arm.latch_calls == 0


def test_wrist_singularity_hold_blocks_the_inward_command_without_latching(monkeypatch):
    from tests.test_damped_ik import make_dls, send_step, Q

    q = Q.copy()
    q[4] = np.deg2rad(0.1)
    controller, arm, receiver = make_dls(q)
    assert send_step(controller, receiver, 4, -0.00001)
    assert controller._singularity_hold
    assert arm.movej_follow_commands == []
    assert arm.slow_stop_calls == 1
    assert arm.latch_calls == 0 and not controller._rearm_required


def test_danger_zone_still_allows_leaving_the_singularity(monkeypatch):
    from tests.test_damped_ik import make_dls, send_step, Q

    q = Q.copy()
    q[4] = np.deg2rad(0.1)
    controller, arm, receiver = make_dls(q)
    assert send_step(controller, receiver, 4, 0.005)
    assert not controller._singularity_hold
    assert arm.movej_follow_commands[-1][4] > q[4]
    assert arm.latch_calls == 0


def test_slowdown_zone_reduces_speed_without_holding(monkeypatch):
    from tests.test_damped_ik import make_dls, send_step, Q

    q = Q.copy()
    q[4] = np.deg2rad(11)
    controller, arm, receiver = make_dls(q)
    controller._diagnostic_row = {}
    assert send_step(controller, receiver, 4, -0.002)
    assert not controller._singularity_hold
    assert 0 < controller._diagnostic_row["singularity_speed_scale"] < 1
    assert len(arm.movej_follow_commands) == 1 and arm.latch_calls == 0


def test_official_ik_hold_sends_nothing_and_latches_no_fault():
    from tests.test_damped_ik import make_dls, send_step, Q

    q = Q.copy()
    q[4] = np.deg2rad(0.1)
    inverse = _IKSolver(error=RuntimeError("global IK must not be called"))
    controller, arm, receiver = make_dls(q, ik_solver=inverse)
    assert send_step(controller, receiver, 4, -0.00001)
    assert controller._singularity_hold and not inverse.calls
    assert arm.movej_follow_commands == [] and not arm.fault_latched


def test_official_ik_leaving_target_is_still_sent():
    from tests.test_damped_ik import make_dls, send_step, Q

    q = Q.copy()
    q[4] = np.deg2rad(0.1)
    inverse = _IKSolver(error=RuntimeError("global IK must not be called"))
    controller, arm, receiver = make_dls(q, ik_solver=inverse)
    assert send_step(controller, receiver, 4, 0.005)
    assert not controller._singularity_hold and not inverse.calls
    assert len(arm.movej_follow_commands) == 1 and not arm.fault_latched


def test_grip_release_clears_the_singularity_speed_scale(monkeypatch):
    from tests.test_damped_ik import make_dls, send_step, Q

    q = Q.copy()
    q[4] = np.deg2rad(0.1)
    controller, arm, receiver = make_dls(q)
    assert send_step(controller, receiver, 4, -0.00001)
    controller.grip_active = True
    controller.xr.grip = 0
    controller._run_cycle(1.0)
    assert not controller._singularity_hold
    assert controller._last_accepted_joint_rad is None
    np.testing.assert_array_equal(controller._dls_velocity, np.zeros(6))
    assert arm.slow_stop_calls >= 1


def test_singularity_avoidance_can_be_disabled_explicitly(monkeypatch):
    feedback = _joint_feedback(WRIST_DANGER_JOINTS)
    controller, arm, _ = _controller(feedback, enable_singularity_avoidance=False)

    assert controller.dls is None

    controller.grip_active = True
    controller.xr.grip = 1.0
    controller.ref_arm_xyz = feedback.tcp_xyz_m.copy()
    controller.ref_arm_quat = feedback.tcp_quat_wxyz.copy()
    controller._last_valid_xr_pose = controller.xr.pose.copy()
    controller._last_valid_xr_t = 0.98
    controller.limiter.reset(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz)
    monkeypatch.setattr(
        controller,
        "_process_xr_pose",
        lambda pose: (np.array([0.002, 0.0, 0.0]), np.zeros(3)),
    )
    monkeypatch.setattr(
        safe_controller_module,
        "apply_delta_pose",
        lambda xyz, quat, delta_xyz, delta_rot: (xyz + delta_xyz, quat.copy()),
    )

    controller._run_cycle(1.0)

    assert controller.limiter.speed_scale == 1.0
    assert not controller._singularity_hold
    assert arm.latch_calls == 0
