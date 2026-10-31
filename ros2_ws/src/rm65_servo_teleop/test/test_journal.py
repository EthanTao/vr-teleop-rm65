"""Persisted journal coverage/accounting tests; no ROS or robot is imported."""

import json
import math
from pathlib import Path

import pytest

from rm65_servo_teleop.journal_node import (
    JournalWriter,
    TOPIC_QOS,
    normalize_diagnostic_values,
    required_topics_for_mode,
)
from rm65_servo_teleop import journal_node

MODES = ("sim", "hardware-dry-run", "hardware")


def metadata(writer):
    return json.loads((writer.path / "session.json").read_text(encoding="utf-8"))


def fill_coverage(writer, *, omit=()):
    for topic in sorted(writer.required_topics - set(omit)):
        assert writer.append(topic, {"fixture": "topic-presence-only"})


def windows_replace_error(code):
    error = PermissionError("injected Windows file conflict")
    error.winerror = code
    return error


@pytest.mark.parametrize("code", [5, 32, 33])
def test_metadata_atomic_replace_recovers_from_transient_windows_conflict(tmp_path, monkeypatch, code):
    source, target = tmp_path / "metadata.tmp", tmp_path / "metadata.json"
    source.write_text('{"new": true}', encoding="utf-8")
    target.write_text('{"old": true}', encoding="utf-8")
    original = Path.replace
    attempts, sleeps = [], []

    def replace(path, destination):
        attempts.append(path)
        if len(attempts) <= 2:
            assert json.loads(target.read_text()) == {"old": True}
            raise windows_replace_error(code)
        return original(path, destination)

    monkeypatch.setattr(Path, "replace", replace)
    monkeypatch.setattr(journal_node.time, "sleep", sleeps.append)
    journal_node.replace_metadata_file(source, target)
    assert json.loads(target.read_text()) == {"new": True}
    assert not source.exists() and len(attempts) == 3 and len(sleeps) == 2


def test_persistent_metadata_conflict_preserves_evidence_and_close_can_retry(tmp_path, monkeypatch):
    writer = JournalWriter(tmp_path, mode="sim")
    fill_coverage(writer)
    old_metadata = (writer.path / "session.json").read_bytes()
    attempts = []
    original = Path.replace

    def denied(path, destination):
        attempts.append(path)
        raise windows_replace_error(5)

    monkeypatch.setattr(Path, "replace", denied)
    monkeypatch.setattr(journal_node.time, "sleep", lambda _: None)
    with pytest.raises(PermissionError):
        writer.close()
    assert len(attempts) == 6
    assert (writer.path / "session.json").read_bytes() == old_metadata
    assert (writer.path / "session.json.tmp").is_file()
    assert not writer.append("/joint_states", {})
    monkeypatch.setattr(Path, "replace", original)
    writer.close()
    assert metadata(writer)["complete"]


@pytest.mark.parametrize("error", [PermissionError("non-Windows permission failure"), OSError("disk failure")])
def test_other_metadata_errors_are_not_retried(tmp_path, monkeypatch, error):
    source, target = tmp_path / "metadata.tmp", tmp_path / "metadata.json"
    source.write_text("new", encoding="utf-8")
    target.write_text("old", encoding="utf-8")
    calls = []

    def fail(path, destination):
        calls.append(path)
        raise error

    monkeypatch.setattr(Path, "replace", fail)
    monkeypatch.setattr(journal_node.time, "sleep", lambda _: pytest.fail("unexpected retry"))
    with pytest.raises(type(error)):
        journal_node.replace_metadata_file(source, target)
    assert len(calls) == 1 and source.read_text() == "new" and target.read_text() == "old"


def test_sensor_subscriptions_accept_best_effort_and_reliable_publishers():
    # DDS requested/offered compatibility: a RELIABLE reader rejects a
    # BEST_EFFORT writer; a BEST_EFFORT reader accepts either reliability.
    for topic in (
        "/rm65/xr/raw",
        "/joint_states",
        "/rm65/transport/joint_states",
        "/rm65/driver/joint_states",
        "/rm65/driver/feedback_snapshot",
        "/rm65_arm_controller/controller_state",
    ):
        policy, depth = TOPIC_QOS[topic]
        assert policy == "best_effort" and depth >= 50
    assert TOPIC_QOS["/rm65/target_pose"][0] == "reliable"
    assert TOPIC_QOS["/rm65/pose_tracking/status"][0] == "reliable"


@pytest.mark.parametrize("mode", MODES)
def test_empty_session_is_never_complete(mode, tmp_path):
    writer = JournalWriter(tmp_path, mode=mode)
    writer.close()
    result = metadata(writer)
    assert result["rows"] == 0 and result["error"] is None and result["dropped"] == 0
    assert not result["complete"] and not result["coverage_complete"]
    assert set(result["missing_required_topics"]) == required_topics_for_mode(mode)


def test_mujoco_session_retains_physics_model_provenance(tmp_path):
    model = {"simulator": "mujoco", "scene_sha256": "fixture", "dynamics_identified": False}
    writer = JournalWriter(tmp_path, mode="sim", simulation_metadata=model)
    fill_coverage(writer)
    writer.close()
    result = metadata(writer)
    assert result["complete"] and result["simulator"] == "mujoco"
    assert json.loads((writer.path / result["simulation_model"]).read_text(encoding="utf-8")) == model


@pytest.mark.parametrize("mode", MODES)
def test_complete_requires_persisted_required_topics_and_exposes_counts(mode, tmp_path):
    writer = JournalWriter(tmp_path, mode=mode)
    fill_coverage(writer)
    assert writer.append("/rm65/xr/raw", {"second": True})
    writer.close()
    result = metadata(writer)
    assert result["mode"] == mode and result["complete"] and result["coverage_complete"]
    assert result["missing_required_topics"] == []
    assert result["rows"] == len(writer.required_topics) + 1
    assert result["topic_counts"]["/rm65/xr/raw"] == {"received": 2, "queued": 2, "written": 2, "dropped": 0}
    assert result["network_loss_checked"] is False
    rows = [json.loads(line) for line in (writer.path / "records.jsonl").read_text(encoding="utf-8").splitlines()]
    assert len([row for row in rows if row["type"] == "sample"]) == result["rows"]
    assert rows[-1]["type"] == "log_end"


@pytest.mark.parametrize(
    "mode,missing",
    [(mode, topic) for mode in MODES for topic in sorted(required_topics_for_mode(mode))],
)
def test_each_missing_required_topic_prevents_complete(mode, missing, tmp_path):
    writer = JournalWriter(tmp_path, mode=mode)
    fill_coverage(writer, omit={missing})
    writer.close()
    result = metadata(writer)
    assert not result["complete"] and not result["coverage_complete"]
    assert result["missing_required_topics"] == [missing]
    assert result["topic_counts"][missing]["written"] == 0


def test_dry_run_has_no_required_send_api_but_live_does(tmp_path):
    dry = required_topics_for_mode("hardware-dry-run")
    live = required_topics_for_mode("hardware")
    assert "/rm65/driver/servo_send_result" not in dry
    assert live - dry == {"/rm65/driver/servo_send_result"}
    assert "/rm65/transport/joint_states" not in required_topics_for_mode("sim")


def test_cached_driver_joint_state_cannot_replace_snapshot_coverage(tmp_path):
    writer = JournalWriter(tmp_path, mode="hardware-dry-run")
    fill_coverage(writer, omit={"/rm65/driver/feedback_snapshot"})
    assert writer.append("/rm65/driver/joint_states", {"position": [0.0] * 6})
    writer.close()
    assert metadata(writer)["missing_required_topics"] == ["/rm65/driver/feedback_snapshot"]


def test_queued_rows_alone_cannot_satisfy_persisted_coverage(tmp_path, monkeypatch):
    import threading

    original_start = threading.Thread.start
    monkeypatch.setattr(threading.Thread, "start", lambda _: None)
    writer = JournalWriter(tmp_path, mode="sim")
    fill_coverage(writer)
    writer._metadata(False)
    result = metadata(writer)
    assert result["topic_counts"]["/rm65/xr/raw"]["queued"] == 1
    assert result["topic_counts"]["/rm65/xr/raw"]["written"] == 0
    assert not result["coverage_complete"]
    original_start(writer._thread)
    writer.close()
    assert metadata(writer)["complete"]


def test_reported_conversion_failure_keeps_fully_covered_session_incomplete(tmp_path):
    writer = JournalWriter(tmp_path, mode="sim")
    fill_coverage(writer)
    writer.fail("message conversion failed on /rm65/xr/raw")
    writer.fail("later failure must not erase first cause")
    writer.close()
    result = metadata(writer)
    assert result["coverage_complete"] and not result["complete"]
    assert result["error"] == "message conversion failed on /rm65/xr/raw"


def test_rejected_post_close_rows_cannot_mutate_final_session(tmp_path):
    writer = JournalWriter(tmp_path, mode="sim")
    fill_coverage(writer)
    writer.close()
    first = metadata(writer)
    assert not writer.append("/rm65/xr/raw", {"late": True})
    writer.close()
    assert metadata(writer) == first


def test_unknown_mode_is_rejected_before_creating_session(tmp_path):
    with pytest.raises(ValueError, match="mode"):
        JournalWriter(tmp_path, mode="misspelled")
    assert list(tmp_path.iterdir()) == []


def test_observed_simulated_joint_state_unknown_effort_is_recorded_without_stopping(tmp_path):
    # Actual sim observation: build/setup/local-servo/servo_observation.json,
    # first_messages['/joint_states']; q/qdot finite, all six effort entries NaN.
    observed = {
        "name": ["joint2", "joint3", "joint1", "joint4", "joint5", "joint6"],
        "position": [0.0, -1.047197551196598, 0.0, 0.0, 1.047197551196598, 0.0],
        "velocity": [0.0] * 6,
        "effort": [math.nan] * 6,
    }
    writer = JournalWriter(tmp_path, mode="sim")
    fill_coverage(writer, omit={"/joint_states"})
    assert writer.append("/joint_states", observed)
    writer.close()
    result = metadata(writer)
    assert result["complete"] and result["quality_valid"] and result["error"] is None
    assert result["optional_nonfinite_fields"] == 6 and result["invalid_rows"] == 0
    rows = [json.loads(line) for line in (writer.path / "records.jsonl").read_text().splitlines()]
    row = next(row for row in rows if row.get("topic") == "/joint_states")
    assert row["value"]["effort"] == [None] * 6
    assert row["value"]["position"] == observed["position"]
    assert row["quality"]["valid"]
    assert row["quality"]["nonfinite_fields"] == [
        {"path": f"/value/effort/{index}", "kind": "NaN", "classification": "optional_diagnostic"} for index in range(6)
    ]
    assert all(math.isnan(value) for value in observed["effort"])


@pytest.mark.parametrize("branch", ["reference", "feedback", "output", "desired", "actual", "error"])
@pytest.mark.parametrize("field", ["accelerations", "effort"])
def test_optional_controller_diagnostics_preserve_nonfinite_types(branch, field):
    value = {branch: {"positions": [0.1], "velocities": [0.0], field: [math.nan, math.inf, -math.inf]}}
    normalized, issues = normalize_diagnostic_values("/rm65_arm_controller/controller_state", value)
    assert normalized[branch][field] == [None, None, None]
    assert [issue["kind"] for issue in issues] == ["NaN", "+Inf", "-Inf"]
    assert [issue["path"] for issue in issues] == [f"/value/{branch}/{field}/{index}" for index in range(3)]
    assert all(issue["classification"] == "optional_diagnostic" for issue in issues)
    json.dumps(normalized, allow_nan=False)


@pytest.mark.parametrize(
    "topic,value,path",
    [
        ("/joint_states", {"position": [math.nan]}, "/value/position/0"),
        ("/rm65/transport/joint_states", {"velocity": [math.inf]}, "/value/velocity/0"),
        ("/rm65/transport/joint_commands", {"position": [-math.inf]}, "/value/position/0"),
        ("/rm65/target_pose", {"pose": {"position": {"x": math.nan}}}, "/value/pose/position/x"),
        ("/rm65/target_pose", {"pose": {"orientation": {"w": math.inf}}}, "/value/pose/orientation/w"),
        (
            "/rm65_arm_controller/controller_state",
            {"feedback": {"velocities": [math.nan]}},
            "/value/feedback/velocities/0",
        ),
        *[
            ("/rm65_arm_controller/joint_trajectory", {"points": [{field: [math.nan]}]}, f"/value/points/0/{field}/0")
            for field in ("positions", "velocities", "accelerations", "effort")
        ],
    ],
)
def test_invalid_control_values_are_preserved_as_invalid_rows_and_cannot_complete(tmp_path, topic, value, path):
    writer = JournalWriter(tmp_path, mode="sim")
    fill_coverage(writer)
    assert writer.append(topic, value)
    writer.close()
    result = metadata(writer)
    assert not result["complete"] and not result["quality_valid"]
    assert result["invalid_rows"] == 1 and "invalid control data" in result["error"]
    assert topic in result["error"] and path in result["error"]
    rows = [json.loads(line) for line in (writer.path / "records.jsonl").read_text().splitlines()]
    bad = next(row for row in rows if row.get("quality", {}).get("valid") is False)
    assert bad["quality"]["nonfinite_fields"][0]["path"] == path
    assert bad["quality"]["nonfinite_fields"][0]["classification"] == "invalid_control"


def test_unclassified_nonfinite_does_not_get_implicit_diagnostic_exemption():
    normalized, issues = normalize_diagnostic_values("/unknown", {"a/b~c": [math.nan]})
    assert normalized == {"a/b~c": [None]}
    assert issues == [{"path": "/value/a~1b~0c/0", "kind": "NaN", "classification": "invalid_control"}]


def test_unsupported_payload_still_fails_writer_and_identifies_topic(tmp_path):
    writer = JournalWriter(tmp_path, mode="sim")
    assert writer.append("/bad/unserializable", {"unsupported": object()})
    writer.close()
    result = metadata(writer)
    assert not result["complete"] and result["rows"] == 0
    assert "journal write failed on /bad/unserializable" in result["error"]
