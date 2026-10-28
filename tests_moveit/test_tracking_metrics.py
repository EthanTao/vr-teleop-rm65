"""Hand-calculable signals verify measurement semantics, without ROS/robot I/O."""

import json
import math
from pathlib import Path

import numpy as np
import pytest

from scripts.tools.analyze_manual_tracking import (
    align_targets,
    analyze_run,
    axis_response,
    main,
    physical_clock_factor,
    quaternion_angle_deg,
    relative_rotvec_deg,
    repeatability,
    sustained_start,
)

IDENTITY = [0.0, 0.0, 0.0, 1.0]


def test_physics_factor_uses_time_progress_and_deduplicates_stamps():
    records = [{"kind": "measurement_start", "host_s": 0.0}]
    records.extend({"kind": "clock", "host_s": i / 100, "stamp_s": i / 50} for i in range(1, 300))
    records.append({"kind": "clock", "host_s": 0.015, "stamp_s": 0.02})
    records.append({"kind": "measurement_end", "host_s": 3.0})
    records.sort(key=lambda row: row["host_s"])
    result = physical_clock_factor(records)
    assert result["real_time_factor"] == pytest.approx(2.0)
    assert result["duplicate_stamps"] == 1
    assert result["unique_physical_stamps"] == 299
    assert result["one_second_window_factor"]["n"] == 3
    assert result["one_second_window_factor"]["p95"] == pytest.approx(2.0)


def test_physics_factor_is_unavailable_without_clock_or_complete_window():
    assert physical_clock_factor([]) is None
    missing = physical_clock_factor([{"kind": "clock", "host_s": 0.0, "stamp_s": 0.0}])
    assert missing["real_time_factor"] is None
    assert missing["reason"] == "measurement_window_missing"


def test_clock_reset_is_reported_instead_of_collapsing_into_speed():
    rows = [
        {"kind": "measurement_start", "host_s": 0.0},
        {"kind": "clock", "host_s": 0.1, "stamp_s": 4.0},
        {"kind": "clock", "host_s": 0.2, "stamp_s": 0.1},
        {"kind": "measurement_end", "host_s": 1.0},
    ]
    result = physical_clock_factor(rows)
    assert result["real_time_factor"] is None
    assert result["reason"] == "physical_time_regressed"


def joint(stamp, x=0.0, *, host=None, angle=0.0):
    return {
        "kind": "joint",
        "host_s": stamp if host is None else host,
        "stamp_s": stamp,
        "receipt_ros_s": stamp + 0.003,
        "q": [angle, 0, 0, 0, 0, 0],
        "dq": [0] * 6,
        "tcp_p": [x, 0, 0],
        "tcp_q": [0, 0, math.sin(angle / 2), math.cos(angle / 2)],
    }


def target(stamp, x=0.0, *, angle=0.0, kind="target"):
    return {
        "kind": kind,
        "host_s": stamp,
        "stamp_s": stamp,
        "p": [x, 0, 0],
        "q": [0, 0, math.sin(angle / 2), math.cos(angle / 2)],
    }


def test_rotation_error_uses_geodesic_angle_and_quaternion_sign_equivalence():
    quarter_turn = [0, 0, math.sqrt(0.5), math.sqrt(0.5)]
    assert quaternion_angle_deg(IDENTITY, quarter_turn) == pytest.approx(90)
    assert quaternion_angle_deg(quarter_turn, -np.array(quarter_turn)) == pytest.approx(0)
    np.testing.assert_allclose(relative_rotvec_deg(IDENTITY, quarter_turn), [0, 0, 90], atol=1e-10)
    np.testing.assert_allclose(relative_rotvec_deg(quarter_turn, IDENTITY), [0, 0, -90], atol=1e-10)


def test_alignment_never_uses_future_target_and_reports_stale_as_missing():
    joints = [joint(10.01), joint(10.03, 0.011), joint(10.15, 0.03)]
    result = align_targets(joints, [target(10.02, 0.01), target(10.04, 0.03)])
    assert result["valid"].tolist() == [False, True, False]
    assert np.isnan(result["position_mm"][0])
    assert result["position_mm"][1] == pytest.approx(1.0)
    assert np.isnan(result["position_mm"][2])
    assert result["age_s"][1] == pytest.approx(0.01)
    assert result["age_s"][2] == pytest.approx(0.11)


def test_alignment_last_equal_stamp_wins_and_unsorted_target_does_not_allow_future():
    result = align_targets([joint(2.02, 0.015)], [target(2.1, 0.04), target(2, 0.01), target(2, 0.02)])
    assert result["valid"].tolist() == [True]
    assert result["position_mm"][0] == pytest.approx(5.0)


def test_sustained_condition_requires_full_interval_and_rejects_sampling_gaps():
    assert sustained_start([0, 0.1, 0.2], [True] * 3, 0.2, max_gap_s=0.11) == pytest.approx(0)
    assert sustained_start([0, 0.1, 0.2], [True, False, True], 0.2, max_gap_s=0.11) is None
    assert sustained_start([0, 0.01, 1.0], [True] * 3, 0.2) is None


def test_axis_response_measures_closed_loop_crossing_and_hold_overshoot():
    move = {"host_s": 1.0, "end_s": 2.0, "axis": 0, "axes": [1, 0, 0, 0, 0, 0]}
    hold = {"host_s": 2.0, "end_s": 3.0}
    targets = [target(0.99), target(1.1, 0.001), target(1.9, 0.01), target(2.9, 0.01)]
    joints = [joint(0.99), joint(1.2, 0.0005), joint(1.3, 0.001), joint(2.2, 0.0115), joint(2.9, 0.01)]
    result = axis_response(move, hold, joints, targets)
    assert result["closed_loop_10pct_delay_s"] == pytest.approx(0.2)
    assert result["published_target_increment"] == pytest.approx(10)
    assert result["hold_axis_overshoot"] == pytest.approx(1.5)


def write_run(path, *, bad_joint=False):
    path.mkdir()
    meta = {
        "run_id": path.name,
        "simulator": "mujoco",
        "completed": True,
        "config": {"model": {"joint_lower_rad": [-1] * 6, "joint_upper_rad": [1] * 6, "joint_velocity_rad_s": [1] * 6}},
    }
    (path / "meta.json").write_text(json.dumps(meta), encoding="utf-8")
    phases = [
        {"kind": "phase", "host_s": 0.0, "name": "x_out_move", "category": "move", "axis": 0},
        {"kind": "phase", "host_s": 0.2, "name": "x_out_hold", "category": "hold", "axis": 0},
        {"kind": "phase", "host_s": 1.2, "name": "release_stop", "category": "release", "axis": None},
        {"kind": "phase", "host_s": 2.2, "name": "idle_end", "category": "idle", "axis": None},
    ]
    rows = phases.copy()
    for i in range(1222):
        t = i * 0.01
        # A known fixed 2 mm / 1 degree error must fail static screening;
        # releasing afterwards must never be counted as target convergence.
        rows += [joint(t), target(t, 0.002, angle=math.radians(1)), target(t, 0.001, kind="internal_target")]
    if bad_joint:
        invalid = joint(0.145)
        invalid["q"][0] = float("nan")
        rows.append(invalid)
    rows.sort(key=lambda row: row["host_s"])
    (path / "raw.jsonl").write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")


def test_static_error_and_internal_clamp_error_are_reported_separately(tmp_path):
    run_dir = tmp_path / "run1"
    write_run(run_dir)
    result, _ = analyze_run(run_dir)
    assert result["status"] == "FAIL"
    hold = next(phase for phase in result["phases"] if phase["category"] == "hold")
    assert hold["tail_position_mm"]["max"] == pytest.approx(2.0)
    assert hold["tail_orientation_deg"]["max"] == pytest.approx(1.0)
    assert hold["settling_time_s"] is None
    assert hold["internal_position_mm"]["max"] == pytest.approx(1.0)
    assert result["moving_position_mm"]["p95"] == pytest.approx(2.0)
    assert result["feedback_receipt_delay_ms"]["p50"] == pytest.approx(3.0)


def test_nonfinite_measurement_cannot_silently_pass_or_break_json_output(tmp_path):
    write_run(tmp_path / "run1", bad_joint=True)
    assert main([str(tmp_path), "--no-plots"]) == 0
    result = json.loads((tmp_path / "analysis/summary.json").read_text(encoding="utf-8"))
    assert result["status"] == "FAIL"
    check = next(c for c in result["runs"][0]["checks"] if c["name"] == "invalid_joint_records")
    assert check["value"] == 1 and check["passed"] is False


def test_target_closure_and_final_tracking_error_are_distinct(tmp_path):
    run_dir = tmp_path / "run1"
    write_run(run_dir)
    records = [json.loads(line) for line in (run_dir / "raw.jsonl").read_text().splitlines()]
    for record in records:
        if record.get("kind") == "phase" and record.get("name") == "x_out_hold":
            record["name"] = "x_back_hold"
    (run_dir / "raw.jsonl").write_text("".join(json.dumps(record) + "\n" for record in records))
    result, _ = analyze_run(run_dir)
    closure = result["return_closure"][0]
    assert closure["target_closure_mm"] == pytest.approx(0.0)
    assert closure["actual_return_mm"] == pytest.approx(0.0)
    assert closure["actual_vs_final_target_mm"] == pytest.approx(2.0)


def test_repeatability_compares_relative_displacements_and_separates_engines():
    def run(name, delta, engine="mujoco"):
        return {
            "run_id": name,
            "simulator": engine,
            "completed": True,
            "repeat_endpoints": [
                {"phase": "x_out_hold", "axis": 0, "position_delta_mm": [delta, 0, 0], "rotation_delta_deg": [0, 0, 0]}
            ],
        }

    result = repeatability([run("one", 10), run("two", 12), run("mock", 100, "mock")])
    actual = next(row for row in result if row["simulator"] == "mujoco")
    assert actual["n"] == 2
    assert actual["position_component_std_mm"][0] == pytest.approx(math.sqrt(2))
    assert actual["position_component_range_mm"] == [2.0, 0.0, 0.0]


def test_repeatability_excludes_incomplete_and_different_measurement_versions():
    base = {
        "simulator": "mujoco",
        "protocol": "full",
        "gui": False,
        "completed": True,
        "repeat_endpoints": [{"phase": "x_out_hold", "position_delta_mm": [1, 0, 0], "rotation_delta_deg": [0, 0, 0]}],
    }
    runs = [
        {**base, "run_id": name, "measurement_sha256": revision, "completed": complete}
        for name, revision, complete in [
            ("one", "a", True),
            ("two", "a", True),
            ("old", "b", True),
            ("failed", "a", False),
        ]
    ]
    result = repeatability(runs)
    same = next(group for group in result if group["measurement_sha256"] == "a")
    assert same["n"] == 2 and same["runs"] == ["one", "two"]
    assert next(group for group in result if group["measurement_sha256"] == "b")["n"] == 1
