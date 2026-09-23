import json
import queue

import numpy as np
import pytest

from tests.test_motion_diagnostics import _row, _self_test
from tests.test_realman_safe_controller import _controller, _feedback
from xrobotoolkit_teleop.hardware.motion_diagnostics import MotionLog
import xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller as controller_module


def test_log_overflow_is_explicit_and_does_not_block_record(tmp_path, monkeypatch):
    path = tmp_path / "overflow.jsonl"
    log = MotionLog(str(path))
    with monkeypatch.context() as patch:

        def full(item):
            raise queue.Full

        patch.setattr(log._queue, "put_nowait", full)
        log.record(_row(1, 1.0))
    assert log.dropped_rows == 1
    with pytest.raises(RuntimeError, match="incomplete motion log"):
        log.close()
    assert json.loads(path.read_text().splitlines()[-1])["dropped_rows"] == 1


def test_disk_failure_is_reported_on_close_after_control_has_stopped(tmp_path):
    log = MotionLog(str(tmp_path / "disk.jsonl"))
    file = log._file

    class BrokenFile:
        def write(self, text):
            raise OSError("injected disk full")

        def close(self):
            file.close()

    log._file = BrokenFile()
    log.record(_row(1, 1.0))
    with pytest.raises(RuntimeError, match="injected disk full"):
        log.close()


def test_nonfinite_sensor_fault_row_remains_valid_json(tmp_path):
    path = tmp_path / "nan.jsonl"
    log = MotionLog(str(path))
    log.record({**_row(1, 1.0), "xr_pose": [float("nan"), float("inf")], "fault_reason": "invalid XR"})
    log.close()
    assert json.loads(path.read_text().splitlines()[0])["xr_pose"] == [None, None]


def test_reset_and_rearm_cycles_are_logged_without_stale_commands(tmp_path, monkeypatch):
    path = tmp_path / "reset.jsonl"
    c, arm, receiver = _controller(_feedback(), motion_log_path=str(path), monotonic_fn=lambda: 1.0)
    monkeypatch.setattr(controller_module, "apply_delta_pose", lambda xyz, quat, delta, rot: (xyz + delta, quat))
    monkeypatch.setattr(controller_module, "quat_diff_as_angle_axis", lambda a, b: np.zeros(3))
    c.home_xyz = receiver.feedback.tcp_xyz_m.copy()
    c.home_quat = receiver.feedback.tcp_quat_wxyz.copy()
    c._start_reset(receiver.feedback)
    c._run_cycle(1.0)
    c._resetting = False
    c._enter_rearm(require_button=True)
    c._run_cycle(1.0)
    c._motion_log.close()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert rows[0]["state"] == "reset"
    assert rows[0]["command_xyz"] is not None
    assert rows[1]["state"] == "rearm"
    assert rows[1]["command_xyz"] is None


def test_failed_send_is_recorded_and_does_not_rearm_itself(tmp_path):
    path = tmp_path / "failed.jsonl"
    c, arm, receiver = _self_test(motion_log_path=str(path))
    arm.send_movep_follow_xyz_quat = lambda *args, **kwargs: False
    c._run_cycle(1.0)
    c._run_cycle(1.02)
    c._motion_log.close()
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert rows[0]["send_result"] is False
    assert rows[0]["state_after"] == "rearm"
    assert rows[1]["send_started_monotonic"] is None
    assert arm.clear_calls == 0


def test_ramp_rejects_distance_over_offset_limit():
    with pytest.raises(ValueError, match="exceeds max_offset"):
        _controller(_feedback(), self_test_ramp=True, max_offset_m=0.01)


def test_dry_run_run_loop_does_not_connect_xr_or_send_final_stop(monkeypatch):
    c, arm, receiver = _self_test(dry_run=True)
    monkeypatch.setattr("builtins.input", lambda prompt: pytest.fail("dry-run needs no confirmation"))
    monkeypatch.setattr(c, "_startup", lambda: setattr(c, "_started", True))
    monkeypatch.setattr(c, "_run_cycle", lambda now: setattr(c, "_self_test_done", True))
    arm.stop = lambda: pytest.fail("dry-run self-test must not send final stop")
    c.run()
    assert not arm.movep_follow_commands


def test_periodic_summary_runs_without_debug_flag(monkeypatch):
    from xrobotoolkit_teleop.hardware.control_timing import FixedRateScheduler

    clock = [1.0]
    c, arm, receiver = _controller(
        _feedback(), monotonic_fn=lambda: clock[0], sleep_fn=lambda delay: clock.__setitem__(0, clock[0] + delay)
    )
    summaries = []

    class Log:
        def summary(self, counters):
            summaries.append((clock[0], counters))

        def close(self):
            pass

    c._motion_log = Log()
    receiver.valid_packet_count = 42
    receiver.invalid_packet_count = 3

    def startup():
        c._started = True
        c.scheduler = FixedRateScheduler(50.0, clock[0])

    def cycle(now):
        if now >= 6.2:
            raise KeyboardInterrupt

    monkeypatch.setattr(c, "_startup", startup)
    monkeypatch.setattr(c, "_run_cycle", cycle)
    c.run()
    assert not c.log_motion_debug
    assert len(summaries) == 2  # one periodic report and one shutdown report
    assert summaries[0][0] >= 6.0
    assert summaries[0][1]["valid_packet_count"] == 42
    assert summaries[0][1]["invalid_packet_count"] == 3
