"""`xrobotoolkit_teleop/utils/motion_log_analysis.py` 的测试。

这些用例的核心是**把「手动了、臂没动」变成可断言的算术**：
预期净位移 0.08 m、实测 0.008 m → 跟随比必须算出 0.1，而且预期没动的轴必须给 `None`
而不是拿一个接近 0 的分母编出一个假数字。
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from xrobotoolkit_teleop.utils import motion_log_analysis as ml


def _row(
    index: int,
    *,
    target=None,
    command=None,
    measured=None,
    status: str | None = "tracking",
    blocked=None,
    direction_ratio=None,
    speed_scale=None,
    lag=None,
    state: str = "teleop",
    dry_run: bool = False,
):
    return {
        "type": "cycle",
        "cycle_index": index,
        "t_monotonic": 100.0 + 0.02 * index,
        "state": state,
        "dry_run": dry_run,
        "target_xyz": target,
        "command_xyz": command,
        "measured_tcp_xyz_m": measured,
        "dls_status": status,
        "singularity_direction_blocked": blocked,
        "singularity_direction_ratio": direction_ratio,
        "singularity_speed_scale": speed_scale,
        "lag_speed_scale": lag,
    }


def _write(tmp_path: Path, rows: list[dict], extra: list[dict] | None = None) -> Path:
    path = tmp_path / "motion.jsonl"
    body = [json.dumps(row) for row in rows]
    body.extend(json.dumps(row) for row in (extra or []))
    path.write_text("\n".join(body) + "\n", encoding="utf-8")
    return path


def test_follow_ratio_pinpoints_the_axis_that_did_not_move():
    """预期 Z 走 8 cm、实测只走 0.8 cm → Z 跟随比 0.1；X/Y 预期没动 → None。"""
    rows = [
        _row(i, target=[0.0, 0.0, 0.02 * i], command=[0.0, 0.0, 0.02 * i], measured=[0.0, 0.0, 0.002 * i])
        for i in range(5)
    ]

    report = ml.segment_report(rows, 1)

    assert report["expected_delta_m"]["Z"] == pytest.approx(0.08)
    assert report["actual_delta_m"]["Z"] == pytest.approx(0.008)
    assert report["follow_ratio"]["Z"] == pytest.approx(0.1)
    assert report["follow_ratio"]["X"] is None
    assert report["follow_ratio"]["Y"] is None
    assert report["follow_ratio_norm"] == pytest.approx(0.1)


def test_reverse_motion_keeps_the_sign_so_it_cannot_hide():
    rows = [
        _row(0, target=[0.0, 0.0, 0.0], command=[0.0, 0.0, 0.0], measured=[0.0, 0.0, 0.0]),
        _row(1, target=[0.0, 0.0, 0.10], command=[0.0, 0.0, 0.10], measured=[0.0, 0.0, -0.01]),
    ]

    report = ml.segment_report(rows, 1)

    assert report["follow_ratio"]["Z"] == pytest.approx(-0.1)


def test_admission_gap_and_servo_gap_are_separated():
    """三层位姿各差多少必须能分别算出来，不能糊成一个数。

    row 0: 预期 0 / 命令 0 / 实测 0
    row 1: 预期 0.01，命令未下发（准入直接挡住）→ 只进「整体」那一项
    row 2: 预期 0.02 / 命令 0.01 / 实测 0.005
           → 准入差距 0.01、伺服差距 0.005、整体差距 0.015
    """
    rows = [
        _row(0, target=[0.0, 0.0, 0.0], command=[0.0, 0.0, 0.0], measured=[0.0, 0.0, 0.0]),
        _row(1, target=[0.0, 0.0, 0.01], command=None, measured=[0.0, 0.0, 0.0], status="hold_singularity"),
        _row(2, target=[0.0, 0.0, 0.02], command=[0.0, 0.0, 0.01], measured=[0.0, 0.0, 0.005]),
    ]

    report = ml.segment_report(rows, 1)

    assert report["admission_gap_m"]["count"] == 2
    assert report["admission_gap_m"]["max"] == pytest.approx(0.01)
    assert report["tracking_gap_m"]["count"] == 2
    assert report["tracking_gap_m"]["max"] == pytest.approx(0.005)
    assert report["expected_gap_m"]["count"] == 3
    assert report["expected_gap_m"]["max"] == pytest.approx(0.015)
    assert report["held_cycles"] == 1


def test_direction_blocked_cycles_are_counted_per_segment():
    rows = [
        _row(0, target=[0.0, 0.0, 0.0], command=[0.0, 0.0, 0.0], measured=[0.0, 0.0, 0.0]),
        _row(
            1, target=[0.0, 0.0, 0.05], command=None, measured=[0.0, 0.0, 0.0], status="hold_singularity", blocked=True
        ),
    ]

    report = ml.segment_report(rows, 1)

    assert report["direction_blocked_cycles"] == 1
    assert report["dls_status"]["hold_singularity"] == 1


def test_segments_split_where_the_target_disappears():
    rows = [
        _row(0, target=[0.0, 0.0, 0.0], command=[0.0, 0.0, 0.0], measured=[0.0, 0.0, 0.0]),
        _row(1, target=[0.0, 0.0, 0.01], command=[0.0, 0.0, 0.01], measured=[0.0, 0.0, 0.001]),
        _row(2),
        _row(3, target=[0.0, 0.0, 0.0], command=[0.0, 0.0, 0.0], measured=[0.0, 0.0, 0.0]),
        _row(4, target=[0.0, 0.0, 0.02], command=[0.0, 0.0, 0.02], measured=[0.0, 0.0, 0.002]),
    ]

    assert [len(segment) for segment in ml.segments(rows)] == [2, 2]


def test_load_cycles_skips_non_cycle_rows_and_reports_bad_lines(tmp_path):
    path = tmp_path / "m.jsonl"
    path.write_text(
        json.dumps({"type": "cycle", "cycle_index": 1})
        + "\n"
        + json.dumps({"type": "summary", "log_dropped_rows": 0})
        + "\n"
        + "{not json}\n"
        + json.dumps({"type": "log_end"})
        + "\n",
        encoding="utf-8",
    )

    cycles, errors = ml.load_cycles(path)

    assert len(cycles) == 1
    assert len(errors) == 1


def test_old_log_without_measured_tcp_is_reported_instead_of_silently_flat(tmp_path):
    """旧版本采集的日志没有实测末端位姿 —— 必须说清「算不了」，不能给一张全零的表。"""
    rows = [_row(i, target=[0.0, 0.0, 0.02 * i], command=[0.0, 0.0, 0.02 * i]) for i in range(3)]
    path = _write(tmp_path, rows)

    report = ml.analyse(path)

    assert report["has_measured_tcp"] is False
    text = ml.format_report(report)
    assert "无法计算" in text
    assert "跟随比" not in text.split("无法计算")[0]


def test_worst_gap_cycles_rank_the_held_report_and_flag_the_missing_command(tmp_path):
    rows = [
        _row(0, target=[0.0, 0.0, 0.0], command=[0.0, 0.0, 0.0], measured=[0.0, 0.0, 0.0]),
        _row(1, target=[0.0, 0.0, 0.02], command=[0.0, 0.0, 0.02], measured=[0.0, 0.0, 0.01]),
        _row(2, target=[0.0, 0.0, 0.30], command=None, measured=[0.0, 0.0, 0.01], status="hold_singularity"),
    ]
    path = _write(tmp_path, rows)

    report = ml.analyse(path)
    worst = report["worst_gap_cycles"]

    assert worst[0]["cycle_index"] == 2
    assert worst[0]["command_sent"] is False
    assert worst[0]["gap_m"] == pytest.approx(0.29)
    assert worst[0]["gap_m"] >= worst[1]["gap_m"]


def test_analyse_counts_cycles_and_keeps_summary_rows_out(tmp_path):
    rows = [
        _row(0, target=[0.0, 0.0, 0.0], command=[0.0, 0.0, 0.0], measured=[0.0, 0.0, 0.0]),
        _row(1, target=[0.0, 0.0, 0.01], command=None, measured=[0.0, 0.0, 0.0], status="hold_singularity"),
    ]
    path = _write(tmp_path, rows, extra=[{"type": "summary", "log_dropped_rows": 0}, {"type": "log_end"}])

    report = ml.analyse(path)

    assert report["total_cycles"] == 2
    assert report["target_cycles"] == 2
    assert report["command_cycles"] == 1
    assert report["measured_cycles"] == 2
    assert report["segment_count"] == 1


def test_main_prints_the_follow_ratio_table(tmp_path, capsys):
    rows = [
        _row(i, target=[0.0, 0.0, 0.02 * i], command=[0.0, 0.0, 0.02 * i], measured=[0.0, 0.0, 0.002 * i])
        for i in range(4)
    ]
    path = _write(tmp_path, rows)

    assert ml.main([str(path)]) == 0
    out = capsys.readouterr().out
    assert "跟随比" in out
    assert "0.10" in out
