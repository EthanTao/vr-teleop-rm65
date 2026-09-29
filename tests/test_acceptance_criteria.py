"""`acceptance_criteria` 的测试：把「验收算不算过」变成可断言的判定。

每一条判据都要有**一个会失败的用例** —— 否则「可判定的判据表」就退化成一句口号。
"""

from __future__ import annotations

import json

import pytest

from xrobotoolkit_teleop.utils import acceptance_criteria as ac


def _segment(cycles: int, ratio: float) -> dict:
    return {"cycles": cycles, "follow_ratio_norm": ratio}


def _motion_report(
    *,
    segments=None,
    duration_s: float = 120.0,
    fault_episodes: int = 0,
    has_measured_tcp: bool = True,
    total_cycles: int = 6000,
) -> dict:
    return {
        "has_measured_tcp": has_measured_tcp,
        "duration_s": duration_s,
        "fault_episodes": fault_episodes,
        "total_cycles": total_cycles,
        "segments": segments if segments is not None else [_segment(100, 0.95), _segment(120, 0.9), _segment(80, 0.85)],
    }


def _mapping_report(*, consistent: bool = True) -> dict:
    return {
        "fits": {
            "measured": {
                "available": True,
                "comparison": {
                    "consistent": consistent,
                    "mismatches": [] if consistent else ["上下(Y)：实测→Y(+) 期望→X(+)，落点轴不同"],
                },
            }
        }
    }


def _by_name(verdict: dict) -> dict[str, dict]:
    return {check["name"]: check for check in verdict["checks"]}


def test_all_criteria_green_passes():
    verdict = ac.evaluate(_motion_report(), _mapping_report())

    assert verdict["passed"] is True
    assert all(check["passed"] for check in verdict["checks"])


def test_missing_measured_tcp_fails_the_record_check():
    verdict = ac.evaluate(_motion_report(has_measured_tcp=False), _mapping_report())

    assert verdict["passed"] is False
    assert _by_name(verdict)["记录带实测末端位姿"]["passed"] is False


def test_short_run_fails_the_duration_check():
    verdict = ac.evaluate(_motion_report(duration_s=5.0), _mapping_report())

    assert verdict["passed"] is False
    assert _by_name(verdict)["整场时长"]["actual"] == 5.0


def test_short_segments_do_not_count_as_effective_samples():
    """一段只有 10 个周期（0.2 s）不算有效样本，光靠段数糊不过去。"""
    motion = _motion_report(segments=[_segment(10, 0.99), _segment(12, 0.99), _segment(8, 0.99)])

    verdict = ac.evaluate(motion, _mapping_report())

    assert _by_name(verdict)["有效段数"]["passed"] is False
    assert _by_name(verdict)["有效段数"]["actual"] == 0


def test_low_follow_ratio_fails_the_median_check():
    """2026-09-26 那种「手在动、臂不动」：跟随比接近 0，必须判不过。"""
    motion = _motion_report(segments=[_segment(100, 0.05), _segment(120, 0.04), _segment(80, 0.06)])

    verdict = ac.evaluate(motion, _mapping_report())

    assert verdict["passed"] is False
    assert _by_name(verdict)["跟随比中位数"]["passed"] is False
    assert _by_name(verdict)["最差段跟随比"]["passed"] is False


def test_a_single_bad_segment_trips_the_worst_segment_floor():
    motion = _motion_report(segments=[_segment(100, 0.95), _segment(120, 0.92), _segment(80, 0.05)])

    verdict = ac.evaluate(motion, _mapping_report())

    assert verdict["passed"] is False
    assert _by_name(verdict)["跟随比中位数"]["passed"] is True  # 中位数仍然很高
    assert _by_name(verdict)["最差段跟随比"]["passed"] is False  # 但硬底线被踩


def test_any_fault_episode_fails_the_latch_check():
    verdict = ac.evaluate(_motion_report(fault_episodes=1), _mapping_report())

    assert verdict["passed"] is False
    assert _by_name(verdict)["故障锁存次数"]["passed"] is False


def test_mapping_mismatch_fails_and_names_the_axis():
    verdict = ac.evaluate(_motion_report(), _mapping_report(consistent=False))

    assert verdict["passed"] is False
    check = _by_name(verdict)["映射方向一致"]
    assert check["passed"] is False
    assert "落点轴不同" in check["note"]


def test_missing_mapping_report_fails_when_it_is_required():
    verdict = ac.evaluate(_motion_report(), None)

    assert verdict["passed"] is False
    assert "未提供映射估计结果" in _by_name(verdict)["映射方向一致"]["note"]


def test_criteria_can_be_relaxed_for_a_first_bench_run():
    motion = _motion_report(segments=[_segment(60, 0.55)], duration_s=20.0, fault_episodes=2)
    relaxed = {
        "min_segments": 1,
        "min_run_seconds": 10.0,
        "min_follow_ratio_p50": 0.5,
        "max_fault_episodes": 3,
        "require_consistent_mapping": False,
    }

    verdict = ac.evaluate(motion, None, relaxed)

    assert verdict["passed"] is True


def test_format_checklist_marks_the_failing_check():
    text = ac.format_checklist(ac.evaluate(_motion_report(fault_episodes=2), _mapping_report()))

    assert "未通过" in text
    assert "❌ 故障锁存次数" in text
    assert "✅ 整场时长" in text


def test_main_returns_two_when_the_run_does_not_pass(tmp_path):
    rows = []
    for index in range(40):
        rows.append(
            {
                "type": "cycle",
                "cycle_index": index,
                "t_monotonic": 1000.0 + 0.02 * index,
                "state": "teleop",
                "dry_run": False,
                "xr_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                "target_xyz": [0.0, 0.0, 0.02 * index],
                "command_xyz": [0.0, 0.0, 0.02 * index],
                "measured_tcp_xyz_m": [0.0, 0.0, 0.0],
                "fault_reason": "跟随误差持续恶化" if index == 30 else None,
                "dls_status": "tracking",
            }
        )
    path = tmp_path / "motion.jsonl"
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")

    assert ac.main([str(path)]) == 2


def _clean_synthetic_rows(segments: int = 3, cycles_per_segment: int = 1200) -> list[dict]:
    """三段干净数据：每段目标往返走动、实测跟到 95%，段间留几帧空档用于切段。"""
    rows: list[dict] = []
    index = 0
    stamp = 1000.0
    for _ in range(segments):
        for step in range(cycles_per_segment):
            target = 0.001 * (step % 100)
            rows.append(
                {
                    "type": "cycle",
                    "cycle_index": index,
                    "t_monotonic": stamp,
                    "state": "teleop",
                    "dry_run": False,
                    "xr_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                    "target_xyz": [0.0, 0.0, target],
                    "command_xyz": [0.0, 0.0, target],
                    "measured_tcp_xyz_m": [0.0, 0.0, target * 0.95],
                    "fault_reason": None,
                    "dls_status": "tracking",
                }
            )
            index += 1
            stamp += 0.02
        for _ in range(3):
            rows.append(
                {
                    "type": "cycle",
                    "cycle_index": index,
                    "t_monotonic": stamp,
                    "state": "idle",
                    "dry_run": False,
                    "xr_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                    "target_xyz": None,
                    "command_xyz": None,
                    "measured_tcp_xyz_m": [0.0, 0.0, 0.0],
                    "fault_reason": None,
                    "dls_status": None,
                }
            )
            index += 1
            stamp += 0.02
    return rows


def test_main_passes_on_a_clean_synthetic_log(tmp_path):
    path = tmp_path / "motion.jsonl"
    path.write_text("\n".join(json.dumps(row) for row in _clean_synthetic_rows()) + "\n", encoding="utf-8")

    # 该日志的手柄位姿恒定（xr_pose 不动），所以映射判据另行关掉
    assert ac.main([str(path), "--no-mapping"]) == 0


def test_no_mapping_flag_also_relaxes_the_mapping_criterion(tmp_path):
    """`--no-mapping` 只跳过计算是不够的：那条判据也必须一起关掉。"""
    path = tmp_path / "motion.jsonl"
    path.write_text("\n".join(json.dumps(row) for row in _clean_synthetic_rows()) + "\n", encoding="utf-8")

    assert ac.main([str(path)]) == 2  # 不关判据 → 判不过
    assert ac.main([str(path), "--no-mapping"]) == 0  # 关掉 → 过得去
