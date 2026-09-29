"""把 `--motion-log-path` 的 JSONL 变成一张「手动了多少 / 臂实际动了多少」的判定表。

为什么需要它
------------
真机验收的现象一旦靠口述描述就无法复核 —— 2026-09-29 那轮就是如此：结论有了，
但没留下任何可以回头核对的原始记录（见 `docs/2026.9.29.md`）。

控制链本身每周期已经落盘三层位姿，本模块只负责把它们对齐、相减、汇总：

===========================  ====================================================
字段                          含义
===========================  ====================================================
``target_xyz``               预期位姿：手柄映射 + 笛卡尔限速之后的「想让它去哪」
``command_xyz``              命令位姿：通过准入后「实际下发的是什么」（保持时为 ``null``）
``measured_tcp_xyz_m``       实测位姿：控制器 UDP 上报的「臂真实在哪」
===========================  ====================================================

两个差值把「不动」拆成两条完全不同的机制：

- ``|target − command|``   **准入**造成的差距（奇异守卫 / 路径守卫 / 跟随余量 / 限速）；
- ``|command − measured|`` **伺服跟随**造成的差距（滞后、被挡、控制器没执行）。

一段（segment）= 连续存在 ``target_xyz`` 的周期，也就是「控制器手里一直有目标」的整段。
每段给出逐轴净位移与**跟随比 = 实测净位移 / 预期净位移**，这正是「手动了臂没动」的量化形式。

边界
----
- ``measured_tcp_xyz_m`` 来自控制器 UDP 上报，与本地官方 FK 不是同一来源，不能互相当作真值。
- 只读文件、纯计算：不连真机、不改任何状态、不参与任何控制判据。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

AXES = ("X", "Y", "Z")

# 净位移小到这个量级时，跟随比没有意义（分母接近 0），标为 None 而不是给一个假数字。
_FOLLOW_RATIO_FLOOR_M = 1e-4

MEASURED_TCP_KEYS = ("measured_tcp_xyz_m", "measured_tcp_quat_wxyz")


def load_cycles(path: str | Path) -> tuple[list[dict], list[str]]:
    """读出所有 ``type == "cycle"`` 的行，并收集解析失败的行号（不抛异常）。"""
    cycles: list[dict] = []
    errors: list[str] = []
    with Path(path).open(encoding="utf-8") as handle:
        for number, raw in enumerate(handle, 1):
            raw = raw.strip()
            if not raw:
                continue
            try:
                row = json.loads(raw)
            except json.JSONDecodeError as exc:
                errors.append(f"第 {number} 行: {exc}")
                continue
            if isinstance(row, dict) and row.get("type") == "cycle":
                cycles.append(row)
    return cycles, errors


def has_measured_tcp(cycles: list[dict]) -> bool:
    """这份日志是否带实测末端位姿（旧版本采集的日志没有它）。"""
    return any(row.get("measured_tcp_xyz_m") is not None for row in cycles)


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


def segments(cycles: list[dict]) -> list[list[dict]]:
    """切段：连续存在 ``target_xyz`` 的周期算一段。"""
    result: list[list[dict]] = []
    current: list[dict] = []
    for row in cycles:
        if _vec3(row, "target_xyz") is None:
            if current:
                result.append(current)
                current = []
            continue
        current.append(row)
    if current:
        result.append(current)
    return result


def _percentiles(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None}
    array = np.asarray(values, dtype=float)
    return {
        "count": int(array.size),
        "p50": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def _pairs(rows: list[dict], key: str) -> list[tuple[float, np.ndarray]]:
    pairs = []
    for row in rows:
        value = _vec3(row, key)
        if value is not None:
            pairs.append((float(row.get("t_monotonic") or 0.0), value))
    return pairs


def segment_report(rows: list[dict], index: int) -> dict:
    """一段的逐轴净位移、跟随比，以及两类差距与拒绝原因分布。"""
    target = _pairs(rows, "target_xyz")
    command = _pairs(rows, "command_xyz")
    measured = _pairs(rows, "measured_tcp_xyz_m")

    admission_gap = [
        float(np.linalg.norm(_vec3(r, "target_xyz") - _vec3(r, "command_xyz")))
        for r in rows
        if _vec3(r, "target_xyz") is not None and _vec3(r, "command_xyz") is not None
    ]
    tracking_gap = [
        float(np.linalg.norm(_vec3(r, "command_xyz") - _vec3(r, "measured_tcp_xyz_m")))
        for r in rows
        if _vec3(r, "command_xyz") is not None and _vec3(r, "measured_tcp_xyz_m") is not None
    ]
    # 不依赖命令层也能算的整体差距：预期 vs 实测。命令被保持时这一项才反映「臂没动」。
    expected_gap = [
        float(np.linalg.norm(_vec3(r, "target_xyz") - _vec3(r, "measured_tcp_xyz_m")))
        for r in rows
        if _vec3(r, "target_xyz") is not None and _vec3(r, "measured_tcp_xyz_m") is not None
    ]

    status_counts: dict[str, int] = {}
    for row in rows:
        status = row.get("dls_status")
        if status is None:
            status = "(none)"
        status_counts[str(status)] = status_counts.get(str(status), 0) + 1

    report: dict = {
        "index": index,
        "cycles": len(rows),
        "states": sorted({str(r.get("state")) for r in rows}),
        "dry_run": rows[0].get("dry_run"),
        "held_cycles": sum(1 for r in rows if r.get("command_xyz") is None),
        "direction_blocked_cycles": sum(1 for r in rows if r.get("singularity_direction_blocked") is True),
        "dls_status": dict(sorted(status_counts.items(), key=lambda item: -item[1])),
        "admission_gap_m": _percentiles(admission_gap),
        "tracking_gap_m": _percentiles(tracking_gap),
        "expected_gap_m": _percentiles(expected_gap),
    }

    if target and len(target) >= 2:
        expected_delta = target[-1][1] - target[0][1]
    else:
        expected_delta = None
    if measured and len(measured) >= 2:
        actual_delta = measured[-1][1] - measured[0][1]
    else:
        actual_delta = None

    def _axis_dict(delta: np.ndarray | None) -> dict[str, float] | None:
        if delta is None:
            return None
        return {axis: float(delta[i]) for i, axis in enumerate(AXES)}

    report["expected_delta_m"] = _axis_dict(expected_delta)
    report["actual_delta_m"] = _axis_dict(actual_delta)
    report["expected_travel_m"] = None if expected_delta is None else float(np.linalg.norm(expected_delta))
    report["actual_travel_m"] = None if actual_delta is None else float(np.linalg.norm(actual_delta))

    if expected_delta is not None and actual_delta is not None:
        ratio: dict[str, float | None] = {}
        for i, axis in enumerate(AXES):
            if abs(expected_delta[i]) < _FOLLOW_RATIO_FLOOR_M:
                ratio[axis] = None
            else:
                ratio[axis] = float(actual_delta[i] / expected_delta[i])
        report["follow_ratio"] = ratio
        norm = float(np.linalg.norm(expected_delta))
        report["follow_ratio_norm"] = (
            None if norm < _FOLLOW_RATIO_FLOOR_M else float(np.linalg.norm(actual_delta) / norm)
        )
    else:
        report["follow_ratio"] = None
        report["follow_ratio_norm"] = None

    report["duration_s"] = None
    if rows and rows[-1].get("t_monotonic") is not None and rows[0].get("t_monotonic") is not None:
        report["duration_s"] = float(rows[-1]["t_monotonic"] - rows[0]["t_monotonic"])
    return report


def analyse(path: str | Path) -> dict:
    """整个日志的汇总：逐段判定表 + 「哪些周期最像『手在动、臂不动』」。"""
    cycles, errors = load_cycles(path)
    report: dict = {
        "path": str(path),
        "parse_errors": errors,
        "total_cycles": len(cycles),
        "has_measured_tcp": has_measured_tcp(cycles),
        "segments": [],
    }
    if not cycles:
        return report

    report["states"] = sorted({str(row.get("state")) for row in cycles})
    report["dry_run"] = sorted({bool(row.get("dry_run")) for row in cycles})
    report["target_cycles"] = sum(1 for row in cycles if _vec3(row, "target_xyz") is not None)
    report["command_cycles"] = sum(1 for row in cycles if _vec3(row, "command_xyz") is not None)
    report["measured_cycles"] = sum(1 for row in cycles if _vec3(row, "measured_tcp_xyz_m") is not None)
    report["fault_cycles"] = sum(1 for row in cycles if row.get("fault_reason"))

    all_segments = segments(cycles)
    report["segment_count"] = len(all_segments)
    report["segments"] = [segment_report(rows, i + 1) for i, rows in enumerate(all_segments)]

    # 最像「手在动、臂不动」的周期：预期与实测差距最大，且当周期没有下发命令。
    gap_rows = []
    for row in cycles:
        target = _vec3(row, "target_xyz")
        measured = _vec3(row, "measured_tcp_xyz_m")
        if target is None or measured is None:
            continue
        gap_rows.append(
            {
                "cycle_index": row.get("cycle_index"),
                "t_monotonic": row.get("t_monotonic"),
                "gap_m": float(np.linalg.norm(target - measured)),
                "command_sent": _vec3(row, "command_xyz") is not None,
                "dls_status": row.get("dls_status"),
                "direction_ratio": row.get("singularity_direction_ratio"),
                "direction_blocked": row.get("singularity_direction_blocked"),
                "speed_scale": row.get("singularity_speed_scale"),
                "lag_speed_scale": row.get("lag_speed_scale"),
                "target_xyz": target.tolist(),
                "measured_tcp_xyz_m": measured.tolist(),
            }
        )
    gap_rows.sort(key=lambda item: -item["gap_m"])
    report["worst_gap_cycles"] = gap_rows
    return report


def format_report(report: dict, *, top: int = 5) -> str:
    """人读输出：先给汇总，再给逐段判定表。"""
    lines: list[str] = []
    lines.append(f"日志: {report['path']}")
    if report["parse_errors"]:
        lines.append(f"  ⚠️ 解析失败 {len(report['parse_errors'])} 行（首个: {report['parse_errors'][0]}）")
    lines.append(
        f"  周期总数 {report['total_cycles']} | 有预期 {report.get('target_cycles', 0)} "
        f"| 有命令 {report.get('command_cycles', 0)} | 有实测末端 {report.get('measured_cycles', 0)}"
        f" | 故障周期 {report.get('fault_cycles', 0)}"
    )
    states = report.get("states")
    if states:
        lines.append(f"  状态: {', '.join(states)}   dry_run={report.get('dry_run')}")

    if not report["has_measured_tcp"]:
        lines.append("")
        lines.append("  ❌ 这份日志没有 `measured_tcp_xyz_m`：采集时用的还是旧版本，**无法计算「臂实际动了多少」**。")
        lines.append("     请用带实测末端位姿的版本重新采集一次（`--motion-log-path`）。")
        return "\n".join(lines)

    lines.append("")
    lines.append(f"共 {report.get('segment_count', 0)} 段（一段 = 控制器手里连续有目标的周期）")
    header = (
        f"  {'段':>3} {'周期':>5} {'秒':>6} {'预期位移cm':>10} {'实测位移cm':>10} "
        f"{'跟随比':>7}  {'保持':>5}  逐轴跟随比 X/Y/Z"
    )
    lines.append(header)
    for seg in report["segments"]:
        expected = seg.get("expected_travel_m")
        actual = seg.get("actual_travel_m")
        ratio = seg.get("follow_ratio") or {}
        ratio_text = "/".join("—" if ratio.get(axis) is None else f"{ratio[axis]:+.2f}" for axis in AXES)
        overall = seg.get("follow_ratio_norm")
        lines.append(
            f"  {seg['index']:>3} {seg['cycles']:>5} "
            f"{(seg.get('duration_s') or 0.0):>6.2f} "
            f"{'—' if expected is None else f'{expected * 100:.1f}':>10} "
            f"{'—' if actual is None else f'{actual * 100:.1f}':>10} "
            f"{'—' if overall is None else f'{overall:.2f}':>7}  "
            f"{seg['held_cycles']:>5}  {ratio_text}"
        )

    lines.append("")
    lines.append("  跟随比 = 实测净位移 / 预期净位移。≈1 表示跟上了；≈0 表示这段基本没动；")
    lines.append("  逐轴比值符号为正说明方向一致，为负说明实测与预期反向（需要单独查）。")

    lines.append("")
    lines.append("  两类差距（把「不动」拆成两条机制）：")
    lines.append(f"  {'段':>3} {'准入 |预期-命令| mm':>22} {'伺服 |命令-实测| mm':>22} {'整体 |预期-实测| mm':>22}")
    for seg in report["segments"]:
        admission = seg["admission_gap_m"]
        tracking = seg["tracking_gap_m"]
        expected = seg["expected_gap_m"]

        def _fmt(stats: dict) -> str:
            if not stats["count"]:
                return "—"
            return f"p50 {stats['p50'] * 1000:.1f} / max {stats['max'] * 1000:.1f}"

        lines.append(f"  {seg['index']:>3} {_fmt(admission):>22} {_fmt(tracking):>22} {_fmt(expected):>22}")

    lines.append("")
    lines.append("  每段的 DLS 结论分布（前 4 项）：")
    for seg in report["segments"]:
        statuses = list(seg["dls_status"].items())[:4]
        text = ", ".join(f"{name}×{count}" for name, count in statuses) or "—"
        lines.append(f"  {seg['index']:>3} {text}")

    worst = report.get("worst_gap_cycles") or []
    if worst:
        lines.append("")
        lines.append(f"  预期与实测差距最大的 {min(top, len(worst))} 个周期：")
        for item in worst[:top]:
            command = "已下发" if item["command_sent"] else "**未下发**"
            lines.append(
                f"    cycle {item['cycle_index']}: 差距 {item['gap_m'] * 1000:.1f} mm, 命令{command}, "
                f"dls={item['dls_status']}, 方向比={item['direction_ratio']}, "
                f"方向受阻={item['direction_blocked']}, 速度缩放={item['speed_scale']}"
            )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    # 输出含中文：终端代码页不是 UTF-8（如 GBK/936）时，未收录的字符会让 print 直接抛
    # UnicodeEncodeError（本模块的第一次演示就被一个 Unicode 减号打崩过）。改成替换而不是
    # 崩溃 —— 一张诊断表降级显示，总好过根本跑不出来。终端按 UTF-8 解码时显示正常；
    # 出现乱码时先 `chcp 65001`，见 docs/2026.9.24.md 的风险边界。
    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(
        description=(
            "把 --motion-log-path 的 JSONL 汇成「手动了多少 / 臂实际动了多少」的判定表。" "只读文件，不连真机。"
        )
    )
    parser.add_argument("log", help="motion log JSONL 路径")
    parser.add_argument("--top", type=int, default=5, help="列出差距最大的前 N 个周期（默认 5）")
    parser.add_argument("--json", action="store_true", help="改为输出 JSON（便于脚本处理）")
    args = parser.parse_args(argv)

    report = analyse(args.log)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(format_report(report, top=max(0, args.top)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
