"""Behavioral tests with no robot, ROS or vendor SDK; fake transport is explicit."""

from dataclasses import replace
import json
from pathlib import Path
import socket

import pytest
import yaml

from rm65_servo_teleop.hand_node import validate_mode
from rm65_servo_teleop.hand_protocol import (
    HandError,
    ReadOnlyRM65,
    decode_words,
    encode_words,
    parse_read,
    position_request,
    probe,
    read_request,
)
from rm65_servo_teleop.hand_session import HandFeedback, HandProfile, HandSession, MockHand
from rm65_servo_teleop.hand_tools import demo, main
from rm65_servo_teleop.journal_node import JournalWriter

PROFILE_PATH = Path(__file__).resolve().parents[1] / "config/hand_mock.yaml"


@pytest.fixture
def profile():
    return HandProfile.from_dict(yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8")))


def feedback(now=0.0, sequence=1, position=(0,) * 6, status=(2,) * 6):
    return HandFeedback(sequence, now, position, status, (0,) * 6)


def command(session, action, now=0.0, **overrides):
    envelope = {
        "session_id": session.session_id,
        "sequence": session.last_command_sequence + 1,
        "expected_revision": session.revision,
        "action": action,
        **overrides,
    }
    return session.command(envelope, now)


def reach(session, hand, stage, start=0.0):
    now = start
    for _ in range(100):
        now = round(now + 0.05, 6)
        session.observe(hand.poll(now), now)
        session.flush_mock(hand, now)
        if session.stage == stage:
            return now
    pytest.fail(f"did not reach {stage}: {session.status(now)}")


def holding(profile):
    hand, session = MockHand(profile), HandSession(profile)
    session.observe(hand.poll(0.0), 0.0)
    assert command(session, "prepare")
    now = reach(session, hand, "ready")
    assert command(session, "grasp", now)
    now = reach(session, hand, "awaiting_grasp_confirmation", now)
    assert command(session, "confirm_grasp", now)
    return session, hand, now


def test_exact_channel_encoding_and_signed_angles():
    words = (0, 1, 255, 256, 32768, 65535)
    assert encode_words(words) == [0, 0, 0, 1, 0, 255, 1, 0, 128, 0, 255, 255]
    assert decode_words(encode_words(words)) == words
    assert decode_words([0, 0, 0, 1, 0, 255, 1, 0, 128, 0, 255, 255], signed=True) == (0, 1, 255, 256, -32768, -1)
    request = position_request(words, 2)
    assert request["address"] == 1135 and request["num"] == 6 and request["port"] == 1


@pytest.mark.parametrize(
    "values", [[], [0] * 5, [0] * 7, [True] * 6, [-1] * 6, [65536] * 6, [1.0] * 6, [float("nan")] * 6]
)
def test_invalid_targets_never_encode(values):
    with pytest.raises(HandError):
        position_request(values, 2)


@pytest.mark.parametrize(
    "response",
    [
        {},
        {"command": "write_registers", "data": [0] * 12},
        {"command": "read_multiple_holding_registers", "data": [0] * 12, "read_state": False},
        {"command": "read_multiple_holding_registers", "data": [0] * 10},
        {"command": "read_multiple_holding_registers", "data": [True] * 12},
        {"command": "read_multiple_holding_registers", "data": [256] * 12},
        {"command": "read_multiple_holding_registers", "data": [0] * 12, "device": 3},
    ],
)
def test_reject_malformed_failed_and_misrouted_replies(response):
    with pytest.raises(HandError):
        parse_read(response, read_request("position", 2))


class FakeSocket:
    def __init__(self, replies):
        self.replies = iter(replies)
        self.data = b""
        self.sent = []
        self.closed = False

    def sendall(self, value):
        self.sent.append(json.loads(value))
        reply = next(self.replies)
        self.data = reply if isinstance(reply, bytes) else json.dumps(reply).encode("ascii") + b"\r\n"

    def settimeout(self, _):
        pass

    def recv(self, _):
        if self.data == b"timeout":
            raise socket.timeout("fixture timeout")
        value, self.data = self.data[:1], self.data[1:]
        return value

    def close(self):
        self.closed = True


def response(values):
    return {"command": "read_multiple_holding_registers", "data": encode_words(values)}


def test_probe_reads_only_verified_identity_and_blocks(monkeypatch):
    connection = FakeSocket([response((0x100, 1, 2, 3, 4, 2))] + [response((0,) * 6)] * 4)
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: connection)
    result = probe("fixture", 2, 1)
    assert connection.closed and len(connection.sent) == 5
    assert {request["command"] for request in connection.sent} == {"read_multiple_holding_registers"}
    assert result["hardware_writes"] is False and result["atomic_snapshot"] is False
    assert result["contact_force_n"] is None and result["grasp_verified"] is False
    assert all(block["clock_source"] == "host_receive_only" for block in result["blocks"].values())


def test_identity_mismatch_stops_before_other_queries(monkeypatch):
    connection = FakeSocket([response((0x200, 1, 2, 3, 4, 2))])
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: connection)
    with pytest.raises(HandError, match="identity"):
        probe("fixture", 2, 1)
    assert connection.closed and len(connection.sent) == 1


@pytest.mark.parametrize(
    "reply",
    [b"timeout", b"", b"{bad}\n", b"x" * 4097, {"command": "wrong"}, response((0,) * 6) | {"read_state": False}],
)
def test_transport_failure_closes_session_and_never_retries(monkeypatch, reply):
    connection = FakeSocket([reply])
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: connection)
    with ReadOnlyRM65("fixture", 2) as client:
        with pytest.raises((OSError, ValueError)):
            client.read("identity")
        assert connection.closed
        with pytest.raises(HandError, match="closed or failed"):
            client.read("position")
        with pytest.raises(HandError, match="instead of reconnecting"):
            client.__enter__()
    assert len(connection.sent) == 1


def test_preview_and_demo_never_open_a_network_connection(monkeypatch, profile, capsys):
    monkeypatch.setattr(socket, "create_connection", lambda *a, **k: pytest.fail("network opened"))
    assert main(["probe", "--arm-ip", "fixture", "--device", "2", "--protocol-major", "1"]) == 0
    assert json.loads(capsys.readouterr().out)["network_opened"] is False
    result = demo(profile)
    assert result[-1]["stage"] == "released" and result[-1]["mock_position_writes"] == 3
    assert result[-1]["physical_grasp_verified"] is False


def test_demo_does_not_overwrite_evidence(tmp_path, capsys):
    output = tmp_path / "evidence.jsonl"
    output.write_text("original", encoding="utf-8")
    assert main(["demo", "--profile", str(PROFILE_PATH), "--output", str(output)]) == 2
    assert output.read_text(encoding="utf-8") == "original"


def test_startup_no_implicit_open_and_unknown_grasp(profile):
    hand, session = MockHand(profile), HandSession(profile)
    session.observe(hand.poll(0.0), 0.0)
    session.flush_mock(hand, 0.0)
    assert hand.write_count == 0 and session.target is None and session.stage == "unknown"
    assert not command(session, "grasp") and not command(session, "confirm_grasp")
    assert not command(session, "begin_insert")


def test_ack_does_not_mean_measured_position_reached(profile):
    hand, session = MockHand(profile), HandSession(profile)
    session.observe(hand.poll(0.0), 0.0)
    assert command(session, "prepare")
    session.flush_mock(hand, 0.0)
    assert session.stage == "preparing" and hand.write_count == 1
    session.observe(feedback(0.01, 2, (4000,) * 6), 0.01)
    assert session.stage == "preparing"


def test_manual_grasp_confirmation_and_no_release_during_insertion(profile):
    session, hand, now = holding(profile)
    assert session.status(now)["grasp_confirmed_by_operator"] is True
    writes_before = hand.write_count
    assert command(session, "begin_insert", now)
    assert not command(session, "release", now)
    session.flush_mock(hand, now)
    now = round(now + 0.05, 6)
    session.observe(hand.poll(now), now)
    assert session.stage == "inserting" and hand.write_count == writes_before
    assert session.status(now)["arm_motion_authorized"] is False
    assert command(session, "finish_insert", now)
    assert command(session, "release", now)
    now = reach(session, hand, "released", now)
    assert hand.write_count == writes_before + 1
    assert session.status(now)["grasp_confirmed_by_operator"] is False


def test_duplicate_feedback_does_not_refresh_and_fault_does_not_open(profile):
    session, hand, now = holding(profile)
    last_sample = session.feedback
    writes_before = hand.write_count
    session.observe(last_sample, now + 0.6)
    session.flush_mock(hand, now + 0.6)
    assert session.stage == "fault" and hand.write_count == writes_before
    assert session.target == profile.grasp
    assert not command(session, "release", now + 0.6)


@pytest.mark.parametrize("status", [3, 4, 5, 6, 65535])
def test_protection_stuck_and_unknown_status_never_mean_grasp_success(profile, status):
    session = HandSession(profile)
    session.observe(feedback(status=(status,) * 6), 0.0)
    assert session.stage == "fault" and not command(session, "confirm_grasp")


def test_position_drift_while_holding_faults_without_sending(profile):
    session, hand, now = holding(profile)
    now += 0.05
    drift = replace(
        session.feedback, sequence=session.feedback.sequence + 1, received_monotonic_s=now, position=(0,) * 6
    )
    session.observe(drift, now)
    session.flush_mock(hand, now)
    assert session.stage == "fault" and hand.write_count == 2


def test_feedback_gap_stays_faulted_after_new_fresh_sample(profile):
    session, hand, now = holding(profile)
    now += 0.8
    session.observe(hand.poll(now), now)
    assert session.stage == "fault" and session.fault_reason == "hand_feedback_stale"


def test_persisted_demo_separates_admission_sent_ack_and_measurement(profile):
    rows = demo(profile)
    kinds = {row["kind"] for row in rows}
    assert {"candidate", "accepted", "sent", "ack", "feedback", "stage", "summary"} <= kinds
    assert [row["stage"] for row in rows if row["kind"] == "stage"] == [
        "preparing",
        "ready",
        "grasping",
        "awaiting_grasp_confirmation",
        "holding",
        "inserting",
        "inserted",
        "releasing",
        "released",
    ]
    assert len([row for row in rows if row["kind"] == "sent"]) == 3


def test_replay_stale_revision_and_previous_session_are_rejected(profile):
    session = HandSession(profile)
    session.observe(feedback(), 0.0)
    assert not command(session, "prepare", sequence=1, expected_revision=123)
    assert not command(session, "prepare", sequence=1)
    assert not command(session, "prepare", sequence=2, session_id="old session")
    assert command(session, "prepare", sequence=3)
    assert not command(session, "prepare", sequence=4, expected_revision=0)


def test_abort_and_restart_never_restore_grasp_or_send_release(profile):
    session, hand, now = holding(profile)
    assert command(session, "abort", now)
    session.flush_mock(hand, now)
    assert session.stage == "fault" and hand.write_count == 2
    restarted = HandSession(profile)
    assert restarted.stage == "unknown" and restarted.session_id != session.session_id
    assert restarted.target is None and restarted.feedback is None


def test_motion_timeout_and_negative_ack_fault_without_retry(profile):
    hand, session = MockHand(profile), HandSession(profile)
    session.observe(feedback(), 0.0)
    assert command(session, "prepare")
    hand.send_position = lambda *args: {"write_state": False, "synthetic": True}
    session.flush_mock(hand, 0.0)
    assert session.stage == "fault" and session.pending_target is None
    another = HandSession(profile)
    another.observe(feedback(position=(4000,) * 6), 0.0)
    assert command(another, "prepare")
    another.flush_mock(MockHand(profile), 0.0)
    for index in range(1, 45):
        another.observe(feedback(index * 0.1, index + 1, position=(4000,) * 6), index * 0.1)
    assert another.stage == "fault" and another.fault_reason == "hand_motion_timeout"


@pytest.mark.parametrize(
    "mode,hand", [("hardware", "mock"), ("hardware-dry-run", "mock"), ("sim", "live"), ("disabled", "mock")]
)
def test_mode_gate_rejects_real_robot_and_live_writes(mode, hand):
    with pytest.raises(ValueError):
        validate_mode(mode, hand)


def test_mock_hand_evidence_is_required_only_when_selected(tmp_path):
    writer = JournalWriter(tmp_path, mode="sim", hand_mode="mock")
    for topic in sorted(writer.required_topics - {"/rm65/hand/status"}):
        writer.append(topic, {"fixture": True})
    writer.close()
    metadata = json.loads((writer.path / "session.json").read_text(encoding="utf-8"))
    assert not metadata["complete"] and metadata["missing_required_topics"] == ["/rm65/hand/status"]
    assert metadata["hand_mode"] == "mock" and metadata["hand_contact_physics_simulated"] is False
    with pytest.raises(ValueError):
        JournalWriter(tmp_path, mode="hardware-dry-run", hand_mode="mock")
