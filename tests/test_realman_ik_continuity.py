"""Host replay of command continuity; no SDK or robot connection."""

from dataclasses import replace

import numpy as np
import pytest

from tests.test_realman_safe_controller import _controller, _feedback
import xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller as controller_module


class IncrementIK:
    def __init__(self):
        self.step = np.deg2rad(0.1)
        self.calls = []
        self.output = None

    def solve(self, xyz, quat, seed):
        self.calls.append((xyz.copy(), quat.copy(), seed.copy()))
        self.output = seed.copy()
        self.output[0] += self.step
        return self.output


def make_controller(**kwargs):
    feedback = replace(_feedback(), joint_rad=np.deg2rad([0, -30, 90, 0, 90, 0]))
    solver = IncrementIK()
    controller, arm, receiver = _controller(
        feedback,
        motion_command="official_ik_movej_follow",
        official_ik_frames_verified=True,
        official_ik_j3_soft_limit_verified=True,
        ik_solver=solver,
        monotonic_fn=lambda: 1.0,
        **kwargs,
    )
    return controller, arm, receiver, solver


def send(controller, receiver):
    feedback = receiver.feedback
    return controller._send_motion(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz, feedback.joint_rad)


@pytest.mark.parametrize("dry_run", [False, True])
def test_next_seed_is_accepted_command_despite_repeated_or_jittered_feedback(dry_run):
    controller, arm, receiver, solver = make_controller(dry_run=dry_run)
    measured = receiver.feedback.joint_rad.copy()
    assert send(controller, receiver)
    first = solver.output.copy()
    receiver.feedback = replace(receiver.feedback, joint_rad=measured + np.deg2rad(0.02))
    assert send(controller, receiver)
    np.testing.assert_allclose(solver.calls[0][2], measured)
    np.testing.assert_allclose(solver.calls[1][2], first)
    np.testing.assert_allclose(controller._last_accepted_joint_rad, solver.output)
    np.testing.assert_allclose(solver.calls[1][0], receiver.feedback.tcp_xyz_m)
    np.testing.assert_allclose(solver.calls[1][1], receiver.feedback.tcp_quat_wxyz)
    assert len(arm.movej_follow_commands) == (0 if dry_run else 2)


def test_continuity_limit_rejects_jump_even_when_close_to_feedback():
    controller, arm, receiver, solver = make_controller()
    solver.step = np.deg2rad(0.3)
    assert send(controller, receiver)
    solver.step = np.deg2rad(-0.6)
    assert not send(controller, receiver)
    assert "command continuity" in arm.fault_reason
    assert len(arm.movej_follow_commands) == 1
    assert controller._last_accepted_joint_rad is None
    assert controller._rearm_button_required


@pytest.mark.parametrize("dry_run", [False, True])
def test_repeated_feedback_cannot_allow_commands_to_run_away(dry_run):
    controller, arm, receiver, solver = make_controller(dry_run=dry_run)
    # Each command step is 0.1 deg, but J1's measured-state budget is 0.36 deg.
    for _ in range(3):
        assert send(controller, receiver)
    assert not send(controller, receiver)
    assert "measured feedback" in arm.fault_reason
    assert len(arm.movej_follow_commands) == (0 if dry_run else 3)
    assert controller._last_accepted_joint_rad is None


@pytest.mark.parametrize("failure", ["false", "exception", "fault_after_send"])
def test_failed_submission_never_commits_candidate_and_requires_rearm(monkeypatch, failure):
    controller, arm, receiver, solver = make_controller()
    assert send(controller, receiver)

    def fail(_joint, active=True):
        if failure == "exception":
            raise OSError("injected send failure")
        if failure == "fault_after_send":
            arm.latch_fault("injected controller fault")
            return True
        return False

    monkeypatch.setattr(arm, "send_movej_follow_joint_rad", fail)
    assert not send(controller, receiver)
    assert arm.fault_latched
    assert controller._last_accepted_joint_rad is None
    assert controller._rearm_button_required
    calls = len(solver.calls)
    assert not send(controller, receiver)
    assert len(solver.calls) == calls


def test_solver_failure_does_not_keep_previous_reference(monkeypatch):
    controller, arm, receiver, solver = make_controller()
    assert send(controller, receiver)

    def fail(*_args):
        raise RuntimeError("injected IK failure")

    monkeypatch.setattr(solver, "solve", fail)
    assert not send(controller, receiver)
    assert controller._last_accepted_joint_rad is None
    assert len(arm.movej_follow_commands) == 1


@pytest.mark.parametrize("bad", [np.full(6, np.nan), np.zeros(5), np.full(6, np.inf)])
def test_invalid_measurement_is_not_hidden_by_cached_seed(bad):
    controller, arm, receiver, solver = make_controller()
    assert send(controller, receiver)
    feedback = receiver.feedback
    assert not controller._send_motion(feedback.tcp_xyz_m, feedback.tcp_quat_wxyz, bad)
    assert len(solver.calls) == 1
    assert controller._last_accepted_joint_rad is None


def test_solver_and_transport_cannot_mutate_continuity_reference(monkeypatch):
    controller, arm, receiver, solver = make_controller()
    assert send(controller, receiver)
    expected = solver.output.copy()
    # A solver may reuse the output buffer on a subsequent call.
    solver.output[:] = 99.0
    np.testing.assert_allclose(controller._last_accepted_joint_rad, expected)

    def mutate_seed(_xyz, _quat, seed):
        target = seed.copy()
        seed[:] = 99.0
        target[0] += np.deg2rad(0.1)
        return target

    def mutate_command(joint, active=True):
        joint[:] = 88.0
        return True

    monkeypatch.setattr(solver, "solve", mutate_seed)
    monkeypatch.setattr(arm, "send_movej_follow_joint_rad", mutate_command)
    assert send(controller, receiver)
    expected[0] += np.deg2rad(0.1)
    np.testing.assert_allclose(controller._last_accepted_joint_rad, expected)
    assert not arm.fault_latched


@pytest.mark.parametrize("boundary", ["release", "fault_rearm", "reset_start", "anchor", "external_fault"])
def test_segment_boundary_rebuilds_seed_from_new_measured_state(boundary):
    controller, arm, receiver, solver = make_controller()
    assert send(controller, receiver)
    old_target = solver.output.copy()
    new_measured = receiver.feedback.joint_rad.copy()
    new_measured[0] += np.deg2rad(2.0)
    receiver.feedback = replace(receiver.feedback, joint_rad=new_measured)

    if boundary == "release":
        controller.grip_active = True
        controller.xr.grip = 0.0
        controller._run_cycle(1.0)
        assert arm.slow_stop_calls == 1
    elif boundary == "fault_rearm":
        controller._latch_safety_fault("injected fault")
        controller._handle_rearm(0.0, False, 1.0)
        assert arm.fault_latched
        controller._handle_rearm(0.0, True, 1.0)
        assert not arm.fault_latched
    elif boundary == "reset_start":
        controller.home_xyz = receiver.feedback.tcp_xyz_m.copy()
        controller.home_quat = receiver.feedback.tcp_quat_wxyz.copy()
        controller._start_reset(receiver.feedback)
        assert controller._resetting
    elif boundary == "external_fault":
        arm.latch_fault("external fault")
        controller._run_cycle(1.0)
        assert controller._last_accepted_joint_rad is None
        controller._handle_rearm(0.0, True, 1.0)
        assert not arm.fault_latched
    else:
        controller._reset_anchor(receiver.feedback)

    assert controller._last_accepted_joint_rad is None
    assert send(controller, receiver)
    np.testing.assert_allclose(solver.calls[-1][2], new_measured)
    assert not np.array_equal(solver.calls[-1][2], old_target)


def test_stale_feedback_clears_reference_before_next_solve():
    controller, arm, receiver, solver = make_controller()
    assert send(controller, receiver)
    controller.grip_active = True
    controller.xr.grip = 1.0
    controller._run_cycle(1.21)
    assert "feedback stale" in arm.fault_reason
    assert len(solver.calls) == 1
    assert controller._last_accepted_joint_rad is None


def test_reset_send_failure_cannot_be_reported_as_success(monkeypatch):
    controller, arm, receiver, solver = make_controller()
    controller.home_xyz = receiver.feedback.tcp_xyz_m.copy()
    controller.home_quat = receiver.feedback.tcp_quat_wxyz.copy()
    controller._start_reset(receiver.feedback)
    controller._reset_step = controller._reset_steps_total
    monkeypatch.setattr(controller_module, "quat_diff_as_angle_axis", lambda *_args: np.zeros(3))
    monkeypatch.setattr(
        controller_module,
        "apply_delta_pose",
        lambda xyz, quat, *_args: (xyz.copy(), quat.copy()),
    )
    monkeypatch.setattr(arm, "send_movej_follow_joint_rad", lambda *_args, **_kwargs: False)
    controller._tick_reset(1.0, receiver.feedback)
    assert arm.fault_latched
    assert not controller._resetting
    assert controller._rearm_button_required
    assert controller._last_accepted_joint_rad is None


def test_joint_diagnostics_distinguish_candidate_and_commit():
    controller, arm, receiver, solver = make_controller()
    controller._diagnostic_row = {}
    assert send(controller, receiver)
    row = controller._diagnostic_row.copy()
    np.testing.assert_allclose(row["joint_measured_rad"], receiver.feedback.joint_rad)
    assert row["joint_prev_accepted_rad"] is None
    np.testing.assert_allclose(row["joint_candidate_rad"], solver.output)
    assert row["joint_target_accepted"] is True

    controller._diagnostic_row = {}
    solver.step = np.deg2rad(1.0)
    assert not send(controller, receiver)
    row = controller._diagnostic_row
    assert row["joint_prev_accepted_rad"] is not None
    assert row["joint_candidate_rad"] is not None
    assert row["joint_target_accepted"] is False


def test_joint_difference_is_not_wrapped_through_a_full_turn(monkeypatch):
    controller, arm, receiver, solver = make_controller()
    measured = receiver.feedback.joint_rad.copy()
    measured[5] = np.deg2rad(359.9)
    receiver.feedback = replace(receiver.feedback, joint_rad=measured)
    target = measured.copy()
    target[5] = np.deg2rad(-359.9)
    monkeypatch.setattr(solver, "solve", lambda *_args: target)
    assert not send(controller, receiver)
    assert arm.fault_latched
    assert not arm.movej_follow_commands


def test_command_reference_sync_failure_clears_joint_reference(monkeypatch):
    controller, arm, receiver, solver = make_controller()
    assert send(controller, receiver)

    def fail(_pose):
        raise RuntimeError("injected local reference failure")

    monkeypatch.setattr(arm, "sync_command_reference", fail)
    assert not send(controller, receiver)
    assert controller._last_accepted_joint_rad is None
    assert arm.fault_latched
    assert controller._rearm_button_required


def test_continuous_replay_tracks_with_small_measured_lag():
    controller, arm, receiver, solver = make_controller()
    measured = receiver.feedback.joint_rad.copy()
    expected = measured.copy()
    targets = []
    for cycle in range(80):
        # Synthetic feedback lags by one command and contains bounded sensor jitter.
        receiver.feedback = replace(
            receiver.feedback,
            joint_rad=expected + np.deg2rad(0.0 if cycle == 0 else 0.01 * (-1) ** cycle),
            received_monotonic_s=1.0 + cycle * controller.dt,
        )
        assert send(controller, receiver)
        np.testing.assert_allclose(solver.calls[-1][2], expected, atol=1e-12)
        expected = expected.copy()
        expected[0] += solver.step
        targets.append(arm.movej_follow_commands[-1])
    np.testing.assert_allclose(np.diff(np.array(targets)[:, 0]), solver.step, atol=1e-12)
    assert not arm.fault_latched
