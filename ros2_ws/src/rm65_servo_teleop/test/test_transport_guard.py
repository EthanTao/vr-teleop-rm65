"""Transport safety behavior with true feedback; these tests never run a robot."""

import json
import math

import pytest

from rm65_servo_teleop.journal_node import JournalWriter
from rm65_servo_teleop.transport_guard import (
    Feedback,
    GuardConfig,
    JOINT_NAMES,
    TransportGuard,
    ordered_joints,
    parse_snapshot,
    parse_stream_reply,
)


def frame(now=10.0, packet=1, **changes):
    values = dict(
        session="driver-one",
        packet_id=packet,
        source_s=now,
        ros_stamp_ns=1_000_000_000,
        position=(0.0,) * 6,
        velocity=(0.0,) * 6,
        controller_errors=(),
        joint_errors=(0,) * 6,
        enabled=(True,) * 6,
    )
    values.update(changes)
    return Feedback(**values)


def active():
    guard = TransportGuard()
    guard.update_feedback(frame(), 10.0)
    guard.update_servo_status(0, 10.0)
    guard.update_enable(True, 10.0)
    assert guard.mark_armed(guard.epoch, True, 10.0)
    return guard


def snapshot(**changes):
    value = dict(
        contract="rm65_udp_callback_v1",
        clock_id="linux-boot",
        session="native-1",
        packet_id=1,
        source_monotonic_ns=10_000_000_000,
        source_ros_ns=1_000_000_000,
        parse_error=0,
        joint_speed_enabled=True,
        speed_unit="rad/s_from_sdk_degree_s",
        joint_names=list(JOINT_NAMES),
        position_rad=[0.0] * 6,
        velocity_rad_s=[0.0] * 6,
        controller_errors=[],
        joint_errors=[0] * 6,
        joint_enabled=[True] * 6,
    )
    value.update(changes)
    return value


def test_names_reorder_all_same_packet_fields():
    names = list(reversed(JOINT_NAMES))
    value = parse_snapshot(
        snapshot(
            joint_names=names,
            position_rad=[6, 5, 4, 3, 2, 1],
            velocity_rad_s=[0.6, 0.5, 0.4, 0.3, 0.2, 0.1],
            joint_errors=[6, 5, 4, 3, 2, 1],
            joint_enabled=[False, True, True, True, True, True],
        ),
        now_s=10.0,
        clock_id="linux-boot",
    )
    assert value.position == (1, 2, 3, 4, 5, 6)
    assert value.velocity == (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
    assert value.joint_errors == (1, 2, 3, 4, 5, 6)
    assert value.enabled == (True, True, True, True, True, False)


@pytest.mark.parametrize(
    "changes",
    [
        {"clock_id": "other-host"},
        {"source_monotonic_ns": 9_000_000_000},
        {"source_monotonic_ns": 11_000_000_000},
        {"packet_id": True},
        {"parse_error": -3},
        {"joint_speed_enabled": False},
        {"velocity_rad_s": []},
        {"speed_unit": "RPM"},
        {"joint_errors": []},
        {"joint_enabled": []},
        {"joint_names": ["joint1"] * 6},
        {"position_rad": [math.nan] * 6},
    ],
)
def test_incomplete_or_unverifiable_snapshot_rejected(changes):
    with pytest.raises((ValueError, TypeError)):
        parse_snapshot(snapshot(**changes), now_s=10.0, clock_id="linux-boot")


def test_replayed_feedback_cannot_refresh_age():
    guard = active()
    assert not guard.update_feedback(frame(now=10.1, packet=1), 10.1)
    assert guard.feedback.source_s == 10.0
    assert guard.latched_reason == "feedback_packet_not_advancing"


def test_packet_counter_does_not_hide_old_receive_time():
    guard = active()
    assert not guard.update_feedback(frame(now=10.0, packet=2), 10.1)
    assert guard.latched_reason == "feedback_receive_time_not_advancing"


def test_driver_restart_latches_even_with_fresh_feedback():
    guard = active()
    guard.update_feedback(frame(now=10.01, session="new-session"), 10.01)
    assert guard.latched_reason == "driver_session_changed"
    assert guard.take_stop_request().kind == "hard"


@pytest.mark.parametrize(
    "field,value,reason",
    [
        ("velocity", (math.radians(18.1), 0, 0, 0, 0, 0), "measured_joint_overspeed"),
        ("position", (4, 0, 0, 0, 0, 0), "measured_joint_limit"),
        ("controller_errors", (7,), "controller_or_joint_error"),
        ("joint_errors", (0, 0, 1, 0, 0, 0), "controller_or_joint_error"),
    ],
)
def test_measured_faults_stop(field, value, reason):
    guard = active()
    guard.update_feedback(frame(10.01, 2, **{field: value}), 10.01)
    assert guard.latched_reason == reason
    assert guard.take_stop_request().kind == "hard"


def test_only_latest_command_no_catchup_queue():
    guard = active()
    guard.update_command(JOINT_NAMES, [0.001] * 6, 10.0)
    guard.update_command(JOINT_NAMES, [0.002] * 6, 10.001)
    assert guard.tick(10.002).target == (0.002,) * 6


def test_first_stream_command_is_bounded_from_actual_feedback():
    guard = active()
    guard.update_command(JOINT_NAMES, [0.01] * 6, 10.0)
    assert guard.tick(10.0).reason == "adjacent_command_step"


def test_command_continuity_and_measured_lead_are_separate_budgets():
    guard = active()
    step = guard.config.speed_rad_s[0] * guard.config.period_s
    for cycle in (1, 2):
        guard.update_command(JOINT_NAMES, [cycle * step, 0, 0, 0, 0, 0], 10 + cycle * 0.02)
        decision = guard.tick(10 + cycle * 0.02)
        assert decision.state == ("submit" if cycle == 1 else "hold")
    # Every adjacent command stays within one cycle, yet tracking stops at two
    # measured cycles and never treats the three-cycle lead as measured error.
    assert decision.reason == "tracking_lead"
    guard.last_submitted = (3 * step, 0, 0, 0, 0, 0)
    guard.update_command(JOINT_NAMES, [3.1 * step, 0, 0, 0, 0, 0], 10.06)
    assert guard.tick(10.06).reason == "tracking_lead_exceeded"


@pytest.mark.parametrize("cause", ["enable", "feedback", "servo", "command"])
def test_independent_timeouts_latch(cause):
    guard = active()
    guard.update_command(JOINT_NAMES, [0] * 6, 10)
    assert guard.tick(10).target is not None
    now = 10.21
    if cause != "enable":
        guard.update_enable(True, now)
    if cause != "feedback":
        guard.update_feedback(frame(now, 2), now)
    if cause != "servo":
        guard.update_servo_status(0, now)
    if cause != "command":
        guard.update_command(JOINT_NAMES, [0] * 6, now)
    assert guard.tick(now).state == "fault"
    assert guard.take_stop_request().kind == "hard"


def test_release_uses_confirmed_soft_stop():
    guard = active()
    guard.update_enable(False, 10.01)
    request = guard.take_stop_request()
    assert request.kind == "soft"
    guard.update_enable(True, 10.02)
    assert not guard.can_arm(10.02)
    assert guard.confirm_stop(request.request_id, True, 10.03)
    assert guard.can_arm(10.03)


def test_fault_recovery_requires_ack_release_and_real_stationarity():
    guard = active()
    guard.fault("test_fault", 10.01)
    request = guard.take_stop_request()
    guard.update_enable(False, 10.02)
    assert guard.tick(10.02).state == "fault"
    guard.confirm_stop(request.request_id, True, 10.03)
    guard.update_feedback(frame(10.04, 2, velocity=(math.radians(1.1), 0, 0, 0, 0, 0)), 10.04)
    assert guard.tick(10.04).state == "fault"
    guard.update_feedback(frame(10.05, 3), 10.05)
    assert guard.tick(10.05).state == "disabled"
    assert not guard.stream_armed
    guard.update_enable(True, 10.06)
    assert guard.can_arm(10.06)


def test_grip_already_released_when_faulted_is_not_a_new_release():
    guard = TransportGuard()
    guard.update_feedback(frame(), 10)
    guard.fault("fault_while_released", 10)
    request = guard.take_stop_request()
    guard.confirm_stop(request.request_id, True, 10.01)
    guard.update_enable(False, 10.02)
    assert guard.tick(10.02).state == "fault"


@pytest.mark.parametrize("timeout", [False, True])
def test_failed_soft_stop_escalates_and_old_ack_cannot_unlock(timeout):
    guard = active()
    guard.update_enable(False, 10.01)
    soft = guard.take_stop_request()
    if timeout:
        guard.tick(10.52)
    else:
        assert not guard.confirm_stop(soft.request_id, False, 10.02)
    hard = guard.take_stop_request()
    assert hard.kind == "hard" and hard.request_id != soft.request_id
    assert not guard.confirm_stop(soft.request_id, True, 10.53 if timeout else 10.03)
    assert not guard.stop_confirmed


def test_hard_stop_without_confirmation_remains_locked():
    guard = active()
    guard.fault("failure", 10.01)
    hard = guard.take_stop_request()
    guard.update_enable(False, 10.02)
    assert not guard.confirm_stop(hard.request_id, False, 10.03)
    guard.update_feedback(frame(10.1, 2), 10.1)
    assert guard.tick(10.1).state == "fault"
    assert not guard.can_arm(10.1)


def test_no_first_command_cannot_wait_forever_after_native_enable():
    guard = active()
    now = 10.21
    guard.update_enable(True, now)
    guard.update_feedback(frame(now, 2), now)
    guard.update_servo_status(0, now)
    assert guard.tick(now).reason == "joint_command_timeout"


def test_slow_vendor_start_does_not_start_a_stop_ack_deadline():
    guard = TransportGuard()
    guard.wait_for_driver()
    guard.update_enable(True, 10)
    assert guard.tick(13).state == "hold"
    assert guard.pending_stop is None and not guard.stream_armed and not guard.stop_confirmed
    assert not guard.begin_driver_handshake(13, services_ready=True)
    guard.update_feedback(frame(13, 1), 13)
    assert not guard.begin_driver_handshake(13, services_ready=False)
    assert guard.begin_driver_handshake(13, services_ready=True)
    request = guard.take_stop_request()
    assert request.kind == "soft" and request.requested_s == 13
    assert guard.confirm_stop(request.request_id, True, 13.1)


def test_startup_fault_stays_closed_and_waits_for_real_service_handshake():
    guard = TransportGuard()
    guard.wait_for_driver()
    guard.fault("invalid_snapshot_before_ready", 10)
    guard.update_enable(True, 10.01)
    guard.update_enable(False, 10.02)
    assert guard.take_stop_request() is None
    assert not guard.can_arm(12)
    guard.update_feedback(frame(13, 1), 13)
    assert guard.begin_driver_handshake(13, services_ready=True)
    request = guard.take_stop_request()
    assert request.kind == "hard" and request.requested_s == 13
    assert not guard.can_arm(13)


def test_stale_snapshot_cannot_release_startup_hold():
    guard = TransportGuard()
    guard.wait_for_driver()
    guard.update_feedback(frame(10, 1), 10)
    assert not guard.begin_driver_handshake(10.3, services_ready=True)
    assert guard.pending_stop is None and guard.startup_waiting


def test_late_enable_epoch_cannot_revive_released_or_faulted_stream():
    guard = active()
    epoch = guard.epoch
    guard.update_enable(False, 10.01)
    assert not guard.mark_armed(epoch, True, 10.02)
    guard.revoke_obsolete_enable(10.02)
    assert guard.take_stop_request().kind == "hard"
    assert not guard.stream_armed


@pytest.mark.parametrize("status", [-1, 2, 4, 5, 99])
def test_humble_halt_codes_latch(status):
    guard = active()
    guard.update_servo_status(status, 10.01)
    assert guard.latched_reason == f"servo_status_{status}"


@pytest.mark.parametrize("status", [0, 1, 3, 6])
def test_humble_deceleration_codes_are_not_misread_as_halts(status):
    guard = active()
    guard.update_servo_status(status, 10.01)
    assert guard.latched_reason is None


@pytest.mark.parametrize(
    "value",
    [
        "0",
        "{}",
        '{"api_code":0,"generation":true,"stream_enabled":true}',
        '{"api_code":0,"generation":0,"stream_enabled":true}',
        '{"api_code":-1,"generation":3,"stream_enabled":true}',
        '{"api_code":0,"generation":3,"stream_enabled":false}',
    ],
)
def test_native_enable_reply_is_strict_json_with_generation(value):
    with pytest.raises((ValueError, TypeError)):
        parse_stream_reply(value, success=True, enabled=True)


def test_native_stop_and_enable_reply():
    assert (
        parse_stream_reply(
            '{"api_code":0,"generation":2,"stream_enabled":false}', success=True, enabled=False
        ).generation
        == 2
    )
    assert (
        parse_stream_reply('{"api_code":0,"generation":2,"stream_enabled":true}', success=True, enabled=True).generation
        == 2
    )


def test_journal_drains_and_distinguishes_simulated_recording(tmp_path):
    from rm65_servo_teleop.journal_node import required_topics_for_mode

    writer = JournalWriter(tmp_path, mode="sim")
    assert writer.append("/joint_states", {"position": [1]}, monotonic_s=10)
    for topic in required_topics_for_mode("sim") - {"/joint_states"}:
        assert writer.append(topic, {"test_fixture": True})
    writer.close()
    metadata = json.loads((writer.path / "session.json").read_text())
    records = [json.loads(row) for row in (writer.path / "records.jsonl").read_text().splitlines()]
    assert metadata["complete"] and metadata["mode"] == "sim"
    assert records[0]["received_monotonic_s"] == 10
    assert records[0]["topic"] == "/joint_states" and records[-1]["type"] == "log_end"


def test_journal_serialization_failure_is_incomplete(tmp_path):
    writer = JournalWriter(tmp_path, mode="hardware-dry-run")
    writer.append("/bad", {"value": math.nan})
    writer.close()
    metadata = json.loads((writer.path / "session.json").read_text())
    assert not metadata["complete"] and metadata["error"]


def test_journal_overflow_is_bounded_and_incomplete(tmp_path, monkeypatch):
    import threading

    original_start = threading.Thread.start
    monkeypatch.setattr(threading.Thread, "start", lambda _: None)
    writer = JournalWriter(tmp_path, mode="sim", capacity=1)
    assert writer.append("/one", 1)
    assert not writer.append("/two", 2)
    assert writer._queue.qsize() == 1
    original_start(writer._thread)
    writer.close()
    metadata = json.loads((writer.path / "session.json").read_text())
    assert metadata["dropped"] == 1 and not metadata["complete"]
