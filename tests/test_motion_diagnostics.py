"""Host-only diagnostics regression; fake transports never contact hardware."""

import json
from dataclasses import replace

import numpy as np
import pytest

from tests.test_realman_safe_controller import _controller, _feedback
import xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller as controller_module
from xrobotoolkit_teleop.hardware.interface.realman_rm65 import RealmanRM65Interface
from xrobotoolkit_teleop.hardware.motion_diagnostics import MotionLog, MotionStreamStats


def _row(index, now, send=None, stamp=1):
    return {
        "type": "cycle",
        "cycle_index": index,
        "t_monotonic": now,
        "send_started_monotonic": send,
        "xr_timestamp_ns": stamp,
    }


def test_batch_log_retains_all_rows_and_shutdown_tail(tmp_path):
    path = tmp_path / "trace.jsonl"
    log = MotionLog(str(path))
    for i in range(1051):
        log.record(_row(i, i * 0.02, i * 0.02))
    log.close()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [r["cycle_index"] for r in rows[:-1]] == list(range(1051))
    assert rows[-1] == {"type": "log_end", "dropped_rows": 0, "error": None}
    with pytest.raises(FileExistsError):
        MotionLog(str(path))


def test_intervals_detect_scheduler_hole_but_exclude_idle_gap():
    stats = MotionStreamStats()
    for row in [
        _row(1, 0, 0, 1),
        _row(2, 0.02, 0.02, 1),
        _row(3, 0.06, 0.06, 2),
        _row(4, 0.08),
        _row(5, 10, 10, 3),
        _row(6, 10.02, 10.02, 3),
    ]:
        stats.observe(row)
    summary = stats.summary()
    assert summary["send_attempt_interval"]["over_30_ms"] == 1
    assert summary["send_attempt_interval"]["max_ms"] == pytest.approx(40)
    assert summary["send_attempt_interval"]["p50_ms"] == pytest.approx(20)
    assert summary["xr_changes_last_second"] == 1
    assert stats.summary()["send_attempt_interval"]["count"] == 0


def test_transport_trace_is_exact_serialized_payload_and_records_failure():
    arm = RealmanRM65Interface()
    arm.motion_diagnostics_enabled = True

    class Socket:
        fail = False

        def sendall(self, data):
            self.data = data
            if self.fail:
                raise OSError("injected failure")

    arm.sock = Socket()
    payload = {"command": "movep_follow", "pose_quat": [300000, 0, 500000, 1000000, 0, 0, 0]}
    arm.send_only(payload)
    assert arm.motion_diagnostic["wire_json"].encode() == arm.sock.data
    assert arm.motion_diagnostic["sendall_completed"]
    arm.sock.fail = True
    with pytest.raises(OSError):
        arm.send_only(payload)
    assert not arm.motion_diagnostic["sendall_completed"]
    assert arm.motion_diagnostic["duration_s"] >= 0


def test_logging_does_not_change_commands_and_records_every_cycle(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(controller_module, "apply_delta_pose", lambda xyz, quat, delta, rot: (xyz + delta, quat.copy()))

    def replay(path):
        clock = [1.0]
        c, arm, receiver = _controller(_feedback(), motion_log_path=path, monotonic_fn=lambda: clock[0])
        c._last_singularity_warning_t = 1.0
        c._run_cycle(1.0)  # idle
        c.grip_active = True
        c.xr.grip = 1.0
        c.ref_arm_xyz = receiver.feedback.tcp_xyz_m.copy()
        c.ref_arm_quat = receiver.feedback.tcp_quat_wxyz.copy()
        c.limiter.reset(c.ref_arm_xyz, c.ref_arm_quat)
        monkeypatch.setattr(c, "_process_xr_pose", lambda pose: (np.array([0.004, 0.0, 0.0]), np.zeros(3)))
        for i in range(1, 101):
            clock[0] = 1 + i * 0.02
            receiver.feedback = _feedback(timestamp=clock[0])
            c.xr.timestamp_ns = i + 1
            c._run_cycle(clock[0])
        c.xr.grip = 0
        c._run_cycle(clock[0])  # release
        if c._motion_log:
            c._motion_log.close()
        return arm.movep_follow_commands

    expected = replay("")
    path = tmp_path / "cycles.jsonl"
    actual = replay(str(path))
    assert len(actual) == len(expected) == 100
    for left, right in zip(actual, expected):
        np.testing.assert_array_equal(left[0], right[0])
        np.testing.assert_array_equal(left[1], right[1])
    rows = [json.loads(line) for line in path.read_text().splitlines()][:-1]
    assert len(rows) == 102
    assert rows[0]["command_xyz"] is None
    assert rows[1]["send_result"] is True
    assert rows[1]["transport"] is None  # fake arm is not evidence of socket transmission
    assert rows[-1]["command_xyz"] is None
    assert "[CMD]" not in capsys.readouterr().out


def _self_test(**kwargs):
    c, arm, receiver = _controller(_feedback(), self_test_ramp=True, max_offset_m=0.03, **kwargs)
    arm._pose_is_within_bounds = lambda xyz: (bool(np.all(np.abs(xyz) < 1)), "bounds")
    c._last_singularity_warning_t = 1.0
    c.xr = None  # any XR access must fail the test
    return c, arm, receiver


@pytest.mark.parametrize("axis", ["+x", "-x", "+y", "-y", "+z", "-z"])
def test_self_test_single_axis_completes_stops_and_never_returns(axis):
    c, arm, receiver = _self_test(self_test_axis=axis)
    origin = receiver.feedback.tcp_xyz_m.copy()
    for i in range(150):
        now = 1 + i * 0.02
        measured = arm.movep_follow_commands[-1][0] if arm.movep_follow_commands else origin
        receiver.feedback = replace(_feedback(timestamp=now), tcp_xyz_m=measured.copy())
        c._run_cycle(now)
        if c._self_test_done:
            break
    assert c._self_test_done
    assert not arm.fault_latched
    assert arm.slow_stop_calls == 1
    commands = np.asarray([xyz for xyz, quat in arm.movep_follow_commands])
    index = "xyz".index(axis[1])
    np.testing.assert_allclose(np.delete(commands - origin, index, axis=1), 0, atol=1e-12)
    assert abs(abs(commands[-1, index] - origin[index]) - 0.02) < 0.001
    assert np.max(np.linalg.norm(np.diff(commands, axis=0), axis=1)) <= 0.02 * c.dt + 1e-12
    count = len(commands)
    c._run_cycle(now + 0.02)
    assert len(arm.movep_follow_commands) == count


def test_self_test_dry_run_generates_targets_without_motion_or_xr(tmp_path):
    path = tmp_path / "ramp.jsonl"
    c, arm, receiver = _self_test(dry_run=True, motion_log_path=str(path))
    for i in range(150):
        now = 1 + i * 0.02
        receiver.feedback = _feedback(timestamp=now)
        c._run_cycle(now)
        if c._self_test_done:
            break
    c._motion_log.close()
    assert c._self_test_done
    assert not arm.movep_follow_commands
    assert arm.slow_stop_calls == 0
    rows = [json.loads(line) for line in path.read_text().splitlines()][:-1]
    assert all(row["xr_pose"] is None and row["transport"] is None for row in rows)
    target = [row["target_xyz"][0] for row in rows if row["target_xyz"] is not None]
    np.testing.assert_allclose(np.diff(target), 0.0004, atol=1e-12)


@pytest.mark.parametrize("fault", ["stale", "speed", "missing_speed", "error", "joint", "thread", "bounds"])
def test_self_test_keeps_feedback_guards_and_latches(fault):
    c, arm, receiver = _self_test()
    if fault == "speed":
        receiver.feedback = _feedback(joint_speed_rad_s=np.ones(6) * 10)
    elif fault == "missing_speed":
        receiver.feedback = replace(_feedback(), joint_speed_rad_s=None)
    elif fault == "error":
        receiver.feedback = replace(_feedback(), arm_error_codes=(7,))
    elif fault == "joint":
        receiver.feedback = replace(_feedback(), joint_rad=np.ones(6) * 100)
    elif fault == "thread":
        receiver.thread_error = "injected receiver failure"
    elif fault == "bounds":
        arm._pose_is_within_bounds = lambda xyz: (False, "bounds")
    c._run_cycle(1.21 if fault == "stale" else 1.0)
    assert arm.fault_latched
    assert c._rearm_required
    assert not arm.movep_follow_commands
    c._run_cycle(1.22)
    assert arm.clear_calls == 0


def test_self_test_keeps_tracking_error_guard(monkeypatch):
    c, arm, _ = _self_test()
    arm.get_last_commanded_pose6 = lambda: [1] * 6
    arm.pose6_to_xyz_quat = lambda pose: (np.array([0.6, 0, 0.5]), np.array([1.0, 0, 0, 0]))
    monkeypatch.setattr(controller_module, "quat_diff_as_angle_axis", lambda a, b: np.zeros(3))
    c._run_cycle(1.0)
    assert "tracking error" in arm.fault_reason
    assert not arm.movep_follow_commands


def test_self_test_cancellation_does_not_connect_or_stop(monkeypatch):
    c, arm, _ = _self_test()
    monkeypatch.setattr("builtins.input", lambda prompt: "no")
    monkeypatch.setattr(c, "_startup", lambda: pytest.fail("must not connect"))
    arm.stop = lambda: pytest.fail("must not send stop before startup")
    c.run()
    assert arm.latch_calls == 0


@pytest.mark.parametrize("kwargs", [{"self_test_axis": "up"}, {"self_test_distance_m": 0.1}])
def test_invalid_ramp_rejected_before_motion(kwargs):
    with pytest.raises(ValueError):
        _self_test(**kwargs)
