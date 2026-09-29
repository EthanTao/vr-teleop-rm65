"""`mapping_estimation` 的测试：把手柄→末端的映射关系变成可断言的算术。

重点不是「拟合器能跑」，而是**当映射被写错时它必须能指出来** ——
2026-09-18 的现场观测（左右→X、上下→Y）与代码推导（上下→X、左右→Y）正好互换，
这几周一直没有定案。这里用一条合成日志把两种情形都钉住。
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from xrobotoolkit_teleop.utils import mapping_estimation as me

# 与 R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE 一致：上下→X、左右→Y、前后→−Z
M_EXPECTED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
# 2026-09-18 现场口述的那一种：左右→X、上下→Y、前后→−Z（X/Y 互换）
M_SWAPPED = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])


def _row(index: int, hand, target=None, measured=None, quat=(0.0, 0.0, 0.0, 1.0)) -> dict:
    pose = [
        float(hand[0]),
        float(hand[1]),
        float(hand[2]),
        float(quat[0]),
        float(quat[1]),
        float(quat[2]),
        float(quat[3]),
    ]
    row = {
        "type": "cycle",
        "cycle_index": index,
        "t_monotonic": 100.0 + 0.02 * index,
        "xr_pose": pose,
        "target_xyz": None if target is None else np.asarray(target, dtype=float).tolist(),
        "measured_tcp_xyz_m": None if measured is None else np.asarray(measured, dtype=float).tolist(),
    }
    return row


def _cycles(matrix, *, count: int = 40, noise: float = 0.0, seed: int = 0, measured_scale=1.0):
    """造一段：手柄各方向小步移动，末端 = 累积的 M·Δ手柄（可加噪）。"""
    rng = np.random.default_rng(seed)
    hand = np.zeros(3)
    target = np.zeros(3)
    measured = np.zeros(3)
    rows = []
    for index in range(count):
        rows.append(_row(index, hand, target, measured))
        step = rng.normal(scale=0.006, size=3)
        hand = hand + step
        target = target + matrix @ step
        measured = measured + matrix @ step * measured_scale
        if noise:
            measured = measured + rng.normal(scale=noise, size=3)
    rows.append(_row(count, hand, target, measured))
    return rows


def test_fit_recovers_a_known_mapping():
    hand, sample = me.collect_pairs(_cycles(M_EXPECTED), "target_xyz")

    fit = me.fit_matrix(hand, sample)

    assert fit is not None
    assert fit["samples"] >= 30
    np.testing.assert_allclose(np.asarray(fit["matrix"]), M_EXPECTED, atol=1e-9)
    assert fit["r2"] == pytest.approx(1.0, abs=1e-9)


def test_axis_verdict_pinpoints_axis_and_sign():
    hand, sample = me.collect_pairs(_cycles(M_EXPECTED), "target_xyz")
    fit = me.fit_matrix(hand, sample)

    verdict = me.axis_verdict(fit["matrix"])

    assert [row["hand_axis"] for row in verdict] == list(me.HAND_AXES)
    assert [row["tcp_axis"] for row in verdict] == ["Y", "X", "Z"]
    assert [row["coefficient"] > 0 for row in verdict] == [True, True, False]
    assert all(row["share"] == pytest.approx(1.0) for row in verdict)


def test_axis_verdict_is_scale_invariant_so_the_limiter_cannot_hide_the_mapping():
    """限速器会把系数整体压小；落点轴与符号必须不变。"""
    verdict_full = me.axis_verdict(M_EXPECTED)
    verdict_small = me.axis_verdict(M_EXPECTED * 0.25)

    assert [(r["tcp_axis"], np.sign(r["coefficient"])) for r in verdict_full] == [
        (r["tcp_axis"], np.sign(r["coefficient"])) for r in verdict_small
    ]


def test_compare_verdicts_flags_the_swapped_axes_from_the_2026_09_18_observation():
    """这条就是对当年那个悬案的可执行判定。"""
    comparison = me.compare_verdicts(me.axis_verdict(M_SWAPPED), me.axis_verdict(M_EXPECTED))

    assert comparison["consistent"] is False
    assert len(comparison["mismatches"]) == 2
    assert "落点轴不同" in comparison["mismatches"][0]


def test_compare_verdicts_accepts_a_match():
    comparison = me.compare_verdicts(me.axis_verdict(M_EXPECTED), me.axis_verdict(M_EXPECTED))

    assert comparison["consistent"] is True
    assert comparison["mismatches"] == []


def test_collect_pairs_drops_hand_steps_below_the_floor():
    rows = [
        _row(0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        _row(1, [0.001, 0.0, 0.0], [0.0, 0.001, 0.0]),  # 1 mm < 2 mm 下限
        _row(2, [0.020, 0.0, 0.0], [0.0, 0.020, 0.0]),  # 19 mm，保留
    ]

    hand, sample = me.collect_pairs(rows, "target_xyz")

    assert hand.shape[0] == 1
    assert hand[0][0] == pytest.approx(0.019)


def test_collect_pairs_drops_samples_with_a_large_hand_rotation():
    """旋转会耦合进平移，姿态变化大的样本必须剔除。"""
    straight = 0.0
    tilted = float(np.deg2rad(10.0))  # 10° 的姿态变化，远超 0.02 rad 门槛
    half = float(np.sin(tilted / 2.0))
    rows = [
        _row(0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], quat=(0.0, 0.0, 0.0, 1.0)),
        _row(1, [0.02, 0.0, 0.0], [0.0, 0.02, 0.0], quat=(0.0, 0.0, half, float(np.cos(tilted / 2.0)))),
        _row(2, [0.04, 0.0, 0.0], [0.0, 0.04, 0.0], quat=(0.0, 0.0, half, float(np.cos(tilted / 2.0)))),
    ]

    hand, _ = me.collect_pairs(rows, "target_xyz")

    # 第一对被 10° 姿态变化剔除，第二对姿态不变、保留
    assert hand.shape[0] == 1
    assert hand[0][0] == pytest.approx(0.02)


def test_collect_pairs_skips_samples_that_are_not_consecutive_cycles():
    rows = [
        _row(0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
        _row(5, [0.05, 0.0, 0.0], [0.0, 0.05, 0.0]),  # 周期号跳了，中间有空洞
    ]

    hand, _ = me.collect_pairs(rows, "target_xyz")

    assert hand.shape[0] == 0


def test_fit_matrix_needs_at_least_three_pairs():
    assert me.fit_matrix(np.zeros((2, 3)), np.zeros((2, 3))) is None
    assert me.fit_matrix(np.zeros((3, 3)), np.zeros((3, 3))) is not None


def test_measured_layer_recovers_the_same_verdict_despite_noise():
    rows = _cycles(M_EXPECTED, count=120, noise=0.0002, seed=7)

    hand, sample = me.collect_pairs(rows, "measured_tcp_xyz_m")
    fit = me.fit_matrix(hand, sample)
    verdict = me.axis_verdict(fit["matrix"])

    assert [row["tcp_axis"] for row in verdict] == ["Y", "X", "Z"]
    assert me.compare_verdicts(verdict, me.axis_verdict(M_EXPECTED))["consistent"] is True


def test_analyse_reports_both_layers_and_a_comparison(tmp_path):
    rows = _cycles(M_EXPECTED, count=60)
    path = tmp_path / "motion.jsonl"
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")

    report = me.analyse(path, expected_matrix=M_EXPECTED)

    assert report["fits"]["target"]["available"] is True
    assert report["fits"]["measured"]["available"] is True
    assert report["fits"]["target"]["comparison"]["consistent"] is True


def test_format_report_says_so_when_there_are_no_samples(tmp_path):
    path = tmp_path / "empty.jsonl"
    path.write_text(json.dumps(_row(0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])) + "\n", encoding="utf-8")

    text = me.format_report(me.analyse(path, expected_matrix=M_EXPECTED))

    assert "样本不足" in text
    assert "--motion-log-path" in text


def test_expected_matrix_matches_the_source_of_truth_where_it_is_importable():
    """容器内（装了 meshcat）核对本模块的副本与 `geometry.py` 真值一致。

    本模块复制那份矩阵是为了在没装 meshcat 的本机也能跑（`geometry.py` 顶层 import meshcat）。
    复制就有漂移风险，所以只要环境允许导入真值，就必须核对。
    """
    pytest.importorskip("meshcat")
    from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE

    np.testing.assert_allclose(
        me.EXPECTED_HARDWARE_MATRIX,
        np.asarray(R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE, dtype=float),
    )


def test_expected_matrix_from_code_always_returns_the_same_matrix():
    """有 meshcat 走真值、没有走副本；两者必须等价（漂移时这条会失败）。"""
    np.testing.assert_allclose(me.expected_matrix_from_code(), me.EXPECTED_HARDWARE_MATRIX)
