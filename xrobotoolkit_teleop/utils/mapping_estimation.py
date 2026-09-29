"""从运动日志**最小二乘估计**「手柄 → 末端」的映射矩阵，把逐轴方向定案变成一张带数字的表。

为什么需要它
------------
「哪根手柄轴对应末端哪根轴、符号对不对」这个问题在项目里争论了几周，一直没有定案：
`docs/RM65参数与坐标系.md` §14.2 的真机观测栏靠口述，2026-09-18 的观测（左右→X、上下→Y）
与代码推导（上下→X、左右→Y）**正好互换**，而两边都没有仪器读数。

但运动日志里其实两层都在：`xr_pose` 是**原始 SDK 手柄位姿**（控制器在 :2015 记的就是
未变换的那个值），`target_xyz` / `measured_tcp_xyz_m` 是末端位姿。于是只要解一个
最小二乘问题就能把映射矩阵拟合出来 —— **不需要操作者打标签，也不需要额外做试动作**：

    Δ末端 ≈ M · Δ手柄

`M` 是 3×3，第 j 列描述「手柄第 j 轴动一点，末端三根轴各动多少」。取每列绝对值最大的那项
就是**该手柄轴的落点轴**，它的符号就是**方向正负**。

代码里的期望值（`realman_rm65_safe_teleop_controller.py:531-572`）：

    Δ_tcp_base = diag(scale) · R · Δ_hand_world
    R = R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE = [[0,1,0],[1,0,0],[0,0,-1]]
    scale = 1.0，deadband = 0，invert_tcp_xy = False（无符号翻转）

⇒ 期望：`Δtcp_x = Δhand_y`（上下→X）、`Δtcp_y = Δhand_x`（左右→Y）、`Δtcp_z = −Δhand_z`。

判读边界
--------
- 笛卡尔限速器会让「命令」滞后于「手柄」，因此拟合出来的**系数会被整体压小**；
  所以判定看的是**落点轴与符号**（对缩放不敏感），不是系数绝对值。
- `target_xyz` 的拟合反映**映射层**；`measured_tcp_xyz_m` 的拟合反映**端到端**（含伺服）。
  两者落点轴一致才说明整链一致。
- 姿态变化大的样本会被剔除，避免旋转耦合污染平移拟合。

本模块只做离线数学：不读串口、不连真机、不改任何状态。
"""

from __future__ import annotations

import numpy as np

from xrobotoolkit_teleop.utils.motion_log_analysis import load_cycles, segments

# 手柄的 SDK 世界系轴名（docs/RM65参数与坐标系.md §13）：X 右 / Y 上 / Z 里
HAND_AXES = ("左右(X)", "上下(Y)", "前后(Z)")
TCP_AXES = ("X", "Y", "Z")

# 单帧手柄位移小于这个值时不入样本：噪声会主导方向。
DEFAULT_MIN_HAND_STEP_M = 0.002
# 单帧手柄姿态变化超过这个值时不入样本：旋转会耦合进平移。
DEFAULT_MAX_ROT_STEP_RAD = 0.02

# 与 `xrobotoolkit_teleop/utils/geometry.py` 的
# `R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE` 一致。
# 这里复制一份，是为了让本工具在**没装 meshcat** 的环境下也能跑 ——
# `geometry.py` 顶层 `import meshcat.transformations`，本机（非容器）导入会失败。
# `tests/test_mapping_estimation.py` 里有一条 `importorskip("meshcat")` 的用例，
# 在能导入 `geometry` 的环境（容器内）核对这份副本与真值一致。
EXPECTED_HARDWARE_MATRIX = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def expected_matrix_from_code() -> np.ndarray:
    """优先用 `geometry.py` 里的真值；导入失败（缺 meshcat）时回落到本模块副本。"""
    try:
        from xrobotoolkit_teleop.utils.geometry import R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE
    except Exception:
        return EXPECTED_HARDWARE_MATRIX
    return np.asarray(R_HEADSET_TO_WORLD_RM65_SIDE_MOUNT_HARDWARE, dtype=float)


def _vec3(row: dict, key: str) -> np.ndarray | None:
    value = row.get(key)
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        return None
    return array


def _hand_xyz(row: dict) -> np.ndarray | None:
    """`xr_pose` = [x, y, z, qx, qy, qz, qw]，取前三个。"""
    value = row.get("xr_pose")
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if array.shape != (7,) or not np.all(np.isfinite(array)):
        return None
    return array[:3]


def _hand_quat_wxyz(row: dict) -> np.ndarray | None:
    value = row.get("xr_pose")
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None
    if array.shape != (7,) or not np.all(np.isfinite(array)):
        return None
    quat = np.array([array[6], array[3], array[4], array[5]], dtype=float)
    norm = float(np.linalg.norm(quat))
    if norm <= 0.0:
        return None
    return quat / norm


def _quat_angle(left: np.ndarray, right: np.ndarray) -> float:
    dot = float(np.clip(abs(float(np.dot(left, right))), 0.0, 1.0))
    return 2.0 * float(np.arccos(dot))


def collect_pairs(
    cycles: list[dict],
    sample_key: str,
    *,
    min_hand_step_m: float = DEFAULT_MIN_HAND_STEP_M,
    max_rot_step_rad: float = DEFAULT_MAX_ROT_STEP_RAD,
) -> tuple[np.ndarray, np.ndarray]:
    """收集 (Δ手柄, Δ末端) 样本对；只取同一段内且周期连续的两帧。"""
    hand: list[np.ndarray] = []
    sample: list[np.ndarray] = []
    for segment in segments(cycles):
        for previous, current in zip(segment, segment[1:]):
            first, second = previous.get("cycle_index"), current.get("cycle_index")
            if first is not None and second is not None and second != first + 1:
                continue
            hand_0, hand_1 = _hand_xyz(previous), _hand_xyz(current)
            sample_0, sample_1 = _vec3(previous, sample_key), _vec3(current, sample_key)
            if hand_0 is None or hand_1 is None or sample_0 is None or sample_1 is None:
                continue
            delta_hand = hand_1 - hand_0
            if float(np.linalg.norm(delta_hand)) < min_hand_step_m:
                continue
            quat_0, quat_1 = _hand_quat_wxyz(previous), _hand_quat_wxyz(current)
            if quat_0 is not None and quat_1 is not None and _quat_angle(quat_0, quat_1) > max_rot_step_rad:
                continue
            hand.append(delta_hand)
            sample.append(sample_1 - sample_0)
    return np.asarray(hand, dtype=float), np.asarray(sample, dtype=float)


def fit_matrix(hand_deltas: np.ndarray, sample_deltas: np.ndarray) -> dict | None:
    """解 `Δ末端 ≈ M · Δ手柄`；样本不足 3 对时返回 None。"""
    hand = np.asarray(hand_deltas, dtype=float)
    sample = np.asarray(sample_deltas, dtype=float)
    if hand.ndim != 2 or hand.shape[1] != 3 or hand.shape[0] < 3 or hand.shape != sample.shape:
        return None
    # lstsq 解 hand @ X = sample，行向量约定 → t = Xᵀ h，故 M = Xᵀ
    solution, _residuals, _rank, _singular = np.linalg.lstsq(hand, sample, rcond=None)
    matrix = solution.T
    residual = sample - hand @ matrix.T
    rms = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    ss_res = float(np.sum(residual**2))
    ss_tot = float(np.sum((sample - sample.mean(axis=0)) ** 2))
    return {
        "matrix": matrix.tolist(),
        "samples": int(hand.shape[0]),
        "residual_rms_m": rms,
        "r2": None if ss_tot <= 0.0 else float(1.0 - ss_res / ss_tot),
    }


def axis_verdict(matrix) -> list[dict]:
    """每根手柄轴的**落点轴与符号**（对整体缩放不敏感，所以能抗限速器的压小）。"""
    array = np.asarray(matrix, dtype=float)
    verdict: list[dict] = []
    for index, hand_axis in enumerate(HAND_AXES):
        column = array[:, index]
        dominant = int(np.argmax(np.abs(column)))
        total = float(np.sum(np.abs(column)))
        verdict.append(
            {
                "hand_axis": hand_axis,
                "tcp_axis": TCP_AXES[dominant],
                "coefficient": float(column[dominant]),
                "share": None if total <= 0.0 else float(abs(column[dominant]) / total),
            }
        )
    return verdict


def compare_verdicts(fitted: list[dict], expected: list[dict]) -> dict:
    """逐轴比较落点轴与符号，给出是否一致与首个不一致项。"""
    mismatches: list[str] = []
    for got, want in zip(fitted, expected):
        same_axis = got["tcp_axis"] == want["tcp_axis"]
        same_sign = bool(np.sign(got["coefficient"]) == np.sign(want["coefficient"]))
        if not (same_axis and same_sign):
            reason = "落点轴不同" if not same_axis else "符号相反"
            mismatches.append(
                f"{got['hand_axis']}：实测→{got['tcp_axis']}"
                f"({'+' if got['coefficient'] >= 0 else '-'}) "
                f"期望→{want['tcp_axis']}({'+' if want['coefficient'] >= 0 else '-'})，{reason}"
            )
    return {"consistent": not mismatches, "mismatches": mismatches}


def analyse(
    path: str,
    *,
    expected_matrix=None,
    min_hand_step_m: float = DEFAULT_MIN_HAND_STEP_M,
    max_rot_step_rad: float = DEFAULT_MAX_ROT_STEP_RAD,
) -> dict:
    """对整个日志做估计；`target_xyz`（映射层）与 `measured_tcp_xyz_m`（端到端）各出一份。"""
    cycles, errors = load_cycles(path)
    report: dict = {
        "path": str(path),
        "parse_errors": errors,
        "total_cycles": len(cycles),
        "cycle_indices": sum(1 for row in cycles if _hand_xyz(row) is not None),
        "fits": {},
    }
    if expected_matrix is not None:
        report["expected"] = {
            "matrix": np.asarray(expected_matrix, dtype=float).tolist(),
            "verdict": axis_verdict(expected_matrix),
        }
    for label, key in (("target", "target_xyz"), ("measured", "measured_tcp_xyz_m")):
        hand, sample = collect_pairs(cycles, key, min_hand_step_m=min_hand_step_m, max_rot_step_rad=max_rot_step_rad)
        fit = fit_matrix(hand, sample)
        if fit is None:
            report["fits"][label] = {"available": False, "samples": int(hand.shape[0])}
            continue
        fit["available"] = True
        fit["verdict"] = axis_verdict(fit["matrix"])
        if "expected" in report:
            fit["comparison"] = compare_verdicts(fit["verdict"], report["expected"]["verdict"])
        report["fits"][label] = fit
    return report


def format_report(report: dict) -> str:
    lines: list[str] = []
    lines.append(f"日志: {report['path']}")
    lines.append(f"  周期总数 {report['total_cycles']}，其中带手柄位姿 {report['cycle_indices']}")
    if report.get("parse_errors"):
        lines.append(f"  ⚠️ 解析失败 {len(report['parse_errors'])} 行")

    if "expected" in report:
        lines.append("")
        lines.append("代码里的期望映射（realman_rm65_safe_teleop_controller.py:531-572）：")
        for row in report["expected"]["verdict"]:
            lines.append(
                f"  {row['hand_axis']:<10} → 末端 {row['tcp_axis']} ({'+' if row['coefficient'] >= 0 else '-'})"
            )

    for label, title in (("target", "映射层（target_xyz，不含伺服）"), ("measured", "端到端（measured_tcp，含伺服）")):
        fit = report["fits"].get(label) or {}
        lines.append("")
        if not fit.get("available"):
            lines.append(f"【{title}】样本不足（{fit.get('samples', 0)} 对），无法估计。")
            lines.append("  需要：一次真机运行 + --motion-log-path，并让手柄在各方向都动一动。")
            continue
        matrix = np.asarray(fit["matrix"], dtype=float)
        lines.append(
            f"【{title}】样本 {fit['samples']} 对，残差 RMS {fit['residual_rms_m'] * 1000:.2f} mm，R² {fit['r2']}"
        )
        lines.append("  拟合矩阵 M（行 = 末端 X/Y/Z，列 = 手柄 " + " / ".join(HAND_AXES) + "）：")
        for index, axis in enumerate(TCP_AXES):
            lines.append("    " + axis + "  " + "  ".join(f"{matrix[index, column]:+.3f}" for column in range(3)))
        lines.append("  落点判定：")
        for row in fit["verdict"]:
            share = "—" if row["share"] is None else f"{row['share'] * 100:.0f}%"
            lines.append(
                f"    {row['hand_axis']:<10} → 末端 {row['tcp_axis']} "
                f"({'+' if row['coefficient'] >= 0 else '-'})，占该列 {share}"
            )
        comparison = fit.get("comparison")
        if comparison:
            lines.append("  与代码期望：" + ("✅ 全部一致" if comparison["consistent"] else "❌ 不一致"))
            for item in comparison["mismatches"]:
                lines.append(f"    · {item}")

    lines.append("")
    lines.append("判读说明：限速器会让系数整体压小，所以看**落点轴与符号**，不看系数绝对值；")
    lines.append("两层（映射层 / 端到端）落点轴一致，才说明整链一致。")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(
        description="从运动日志最小二乘估计「手柄 → 末端」映射矩阵，用于逐轴方向定案。只读，不连真机。"
    )
    parser.add_argument("log", help="motion log JSONL 路径")
    parser.add_argument("--min-hand-step-mm", type=float, default=DEFAULT_MIN_HAND_STEP_M * 1000.0)
    parser.add_argument("--max-rot-step-deg", type=float, default=None)
    parser.add_argument("--json", action="store_true", help="输出 JSON")
    args = parser.parse_args(argv)

    max_rot = DEFAULT_MAX_ROT_STEP_RAD
    if args.max_rot_step_deg is not None:
        max_rot = float(np.deg2rad(args.max_rot_step_deg))

    report = analyse(
        args.log,
        expected_matrix=expected_matrix_from_code(),
        min_hand_step_m=max(0.0, args.min_hand_step_mm / 1000.0),
        max_rot_step_rad=max(0.0, max_rot),
    )
    if args.json:
        import json

        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(format_report(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
