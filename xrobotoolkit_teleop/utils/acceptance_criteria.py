"""真机遥操作验收的**量化判据**与判定。

为什么需要它
------------
2026-09-29 的诊断结论是：这几周的验收**全部靠口述**，所以每轮都说不清、每轮都要重开一轮
讨论（`docs/2026.9.29.md`）。「工程化落地」的第一步不是把算法调得更好，而是
**把「算不算过」变成数字**。

本模块把验收写成一张可判定的清单：给定一份 `--motion-log-path` 的 JSONL，
输出每条判据的实际值、阈值与 ✅ / ❌。

判据从哪来
----------
- **跟随比**：`motion_log_analysis` 给出的「实测净位移 / 预期净位移」（每段一个数）；
- **锁存**：`fault_reason` 由空变非空的**事件次数**（不是周期数 —— 锁存是持续状态）；
- **方向**：`mapping_estimation` 的落点轴 + 符号是否与代码期望一致；
- **样本量**：有效段数与整场时长，避免「跑了 3 秒就算过」。

默认阈值怎么定的
----------------
阈值本身是**工程约定**，不是物理常量，可按现场调整（`criteria` 参数覆盖）：

- `min_follow_ratio_p50 = 0.80`：一段的实测净位移达到预期净位移的 80%。
  低于这个值，「手动了、臂几乎没动」就成立，数据不可用于采数。
- `max_fault_episodes = 0`：验收期间不允许出现故障锁存 —— 一次锁存就要人工介入，
  那已经不是「可交付」的状态。
- `min_run_seconds = 60` / `min_segments = 3` / `min_segment_cycles = 50`：
  保证样本不是几秒钟的偶然。

边界
----
- 只做**离线判定**：读日志、算判据；不连真机、不改任何状态、不参与控制判据。
- 判据为「过 / 不过」，**不替代**逐轴方向的人工确认与现场安全监护。
"""

from __future__ import annotations

DEFAULT_CRITERIA: dict[str, float | int | bool] = {
    # 证据必须完整：日志正常收尾、无丢行、无写入错误。放在第一条 ——
    # 一份被截断的日志看起来与完整的一模一样，先确认证据可信再谈数字。
    "require_complete_log": True,
    # 必须是真实运行：--dry-run 下机械臂根本不运动，跟随比必然接近 0，
    # 不提这一条的话判据会给出满屏误导性的 ❌，而不是「这不是一次运动验收」。
    "require_live_run": True,
    # 记录必须带实测末端位姿，否则「臂实际动了多少」无从计算
    "require_measured_tcp": True,
    # 整场最短时长与有效段
    "min_run_seconds": 60.0,
    "min_segments": 3,
    "min_segment_cycles": 50,
    # 跟随比：有效段的中位数下限，以及任何一段都不低于的硬底线
    "min_follow_ratio_p50": 0.80,
    "min_follow_ratio_worst": 0.30,
    # 故障锁存次数上限（一次锁存就要人工介入）
    "max_fault_episodes": 0,
    # 映射落点轴必须与代码期望一致
    "require_consistent_mapping": True,
}


def _check(name: str, passed: bool, actual, threshold, note: str = "") -> dict:
    return {
        "name": name,
        "passed": bool(passed),
        "actual": actual,
        "threshold": threshold,
        "note": note,
    }


def evaluate(
    motion_report: dict,
    mapping_report: dict | None = None,
    criteria: dict | None = None,
) -> dict:
    """按判据表判定一次验收；返回 `passed` 与逐条 `checks`。"""
    limits = dict(DEFAULT_CRITERIA)
    if criteria:
        limits.update(criteria)

    checks: list[dict] = []

    if limits["require_complete_log"]:
        complete = bool(motion_report.get("log_complete"))
        reasons: list[str] = []
        if motion_report.get("log_end") is None:
            reasons.append("没有 log_end 收尾行（进程未正常退出，日志可能被截断）")
        if motion_report.get("log_dropped_rows"):
            reasons.append(f"丢行 {motion_report['log_dropped_rows']} 行")
        if motion_report.get("log_error"):
            reasons.append(f"写入错误：{motion_report['log_error']}")
        checks.append(
            _check("日志证据完整", complete, complete, True, "；".join(reasons) or "正常收尾、无丢行、无写入错误")
        )

    if limits["require_live_run"]:
        flags = motion_report.get("dry_run")
        # `analyse()` 把它记成「出现过的取值集合」，例如 [False] 或 [True]。
        is_live = flags is not None and True not in list(flags)
        checks.append(
            _check(
                "为真实运行（非 dry-run）",
                is_live,
                "live" if is_live else "dry-run",
                "live",
                "" if is_live else "本次是 --dry-run：机械臂没有运动，跟随比必然接近 0，不能作为运动验收",
            )
        )

    if limits["require_measured_tcp"]:
        has_tcp = bool(motion_report.get("has_measured_tcp"))
        checks.append(
            _check(
                "记录带实测末端位姿",
                has_tcp,
                has_tcp,
                True,
                "旧版本采集的日志没有 measured_tcp_xyz_m，无法计算跟随比",
            )
        )

    duration = motion_report.get("duration_s")
    checks.append(
        _check(
            "整场时长",
            duration is not None and float(duration) >= float(limits["min_run_seconds"]),
            None if duration is None else round(float(duration), 2),
            f">= {limits['min_run_seconds']} s",
        )
    )

    all_segments = motion_report.get("segments") or []
    effective = [
        segment for segment in all_segments if int(segment.get("cycles", 0)) >= int(limits["min_segment_cycles"])
    ]
    checks.append(
        _check(
            "有效段数",
            len(effective) >= int(limits["min_segments"]),
            len(effective),
            f">= {limits['min_segments']}",
            f"有效段 = 连续有目标且不少于 {limits['min_segment_cycles']} 个周期的段",
        )
    )

    ratios = [
        float(segment["follow_ratio_norm"]) for segment in effective if segment.get("follow_ratio_norm") is not None
    ]
    ratios.sort()
    if ratios:
        middle = (
            ratios[len(ratios) // 2]
            if len(ratios) % 2
            else 0.5 * (ratios[len(ratios) // 2 - 1] + ratios[len(ratios) // 2])
        )
        checks.append(
            _check(
                "跟随比中位数",
                middle >= float(limits["min_follow_ratio_p50"]),
                round(middle, 3),
                f">= {limits['min_follow_ratio_p50']}",
                "跟随比 = 实测净位移 / 预期净位移；1.0 表示完全跟上",
            )
        )
        checks.append(
            _check(
                "最差段跟随比",
                ratios[0] >= float(limits["min_follow_ratio_worst"]),
                round(ratios[0], 3),
                f">= {limits['min_follow_ratio_worst']}",
                "任何一段都不允许低于硬底线（低于此值说明该段基本没动）",
            )
        )
    else:
        checks.append(_check("跟随比中位数", False, None, f">= {limits['min_follow_ratio_p50']}", "没有可用样本"))

    episodes = int(motion_report.get("fault_episodes") or 0)
    checks.append(
        _check(
            "故障锁存次数",
            episodes <= int(limits["max_fault_episodes"]),
            episodes,
            f"<= {limits['max_fault_episodes']}",
            "锁存是持续状态，这里数的是「由正常变锁存」的事件次数",
        )
    )

    if limits["require_consistent_mapping"]:
        if mapping_report is None:
            checks.append(_check("映射方向一致", False, None, True, "未提供映射估计结果"))
        else:
            fit = (mapping_report.get("fits") or {}).get("measured") or {}
            comparison = fit.get("comparison") or {}
            consistent = bool(comparison.get("consistent"))
            checks.append(
                _check(
                    "映射方向一致",
                    consistent,
                    consistent,
                    True,
                    "；".join(comparison.get("mismatches") or []) or "落点轴与符号均与代码期望一致",
                )
            )

    return {
        "passed": all(check["passed"] for check in checks),
        "checks": checks,
        "criteria": limits,
    }


def format_checklist(report: dict) -> str:
    lines: list[str] = []
    lines.append("验收判定：" + ("✅ 通过" if report["passed"] else "❌ 未通过"))
    lines.append("")
    for check in report["checks"]:
        mark = "✅" if check["passed"] else "❌"
        lines.append(f"  {mark} {check['name']}：实际 {check['actual']}，要求 {check['threshold']}")
        if check["note"]:
            lines.append(f"      {check['note']}")
    lines.append("")
    lines.append("判据是工程约定、不是物理常量，可按现场调整；")
    lines.append("判定为「过/不过」，不替代逐轴方向的人工确认与现场安全监护。")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    import argparse
    import sys

    try:
        sys.stdout.reconfigure(errors="replace")
    except (AttributeError, OSError):
        pass

    parser = argparse.ArgumentParser(
        description="对一次真机验收做量化判定：读 --motion-log-path 的 JSONL，输出判据清单。只读，不连真机。"
    )
    parser.add_argument("log", help="motion log JSONL 路径")
    parser.add_argument("--no-mapping", action="store_true", help="跳过映射方向判据（该日志没有手柄位姿时）")
    parser.add_argument("--min-run-seconds", type=float, default=None)
    parser.add_argument("--min-follow-ratio", type=float, default=None)
    parser.add_argument("--json", action="store_true", help="输出 JSON（便于脚本消费；未通过时退出码为 2）")
    args = parser.parse_args(argv)

    from xrobotoolkit_teleop.utils import mapping_estimation
    from xrobotoolkit_teleop.utils import motion_log_analysis

    motion_report = motion_log_analysis.analyse(args.log)
    mapping_report = None
    if not args.no_mapping:
        mapping_report = mapping_estimation.analyse(
            args.log, expected_matrix=mapping_estimation.expected_matrix_from_code()
        )

    criteria: dict = {}
    if args.no_mapping:
        # 跳过映射估计时，必须同时关掉那条判据 —— 否则「不算」会被判成「不一致」。
        criteria["require_consistent_mapping"] = False
    if args.min_run_seconds is not None:
        criteria["min_run_seconds"] = args.min_run_seconds
    if args.min_follow_ratio is not None:
        criteria["min_follow_ratio_p50"] = args.min_follow_ratio

    verdict = evaluate(motion_report, mapping_report, criteria or None)

    if args.json:
        import json

        print(
            json.dumps(
                {
                    "verdict": verdict,
                    "motion": {
                        key: motion_report.get(key)
                        for key in ("total_cycles", "duration_s", "fault_episodes", "segment_count", "has_measured_tcp")
                    },
                    "mapping": (
                        None
                        if mapping_report is None
                        else {
                            label: {
                                "available": fit.get("available"),
                                "samples": fit.get("samples"),
                                "verdict": fit.get("verdict"),
                                "comparison": fit.get("comparison"),
                            }
                            for label, fit in (mapping_report.get("fits") or {}).items()
                        }
                    ),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        print(motion_log_analysis.format_report(motion_report))
        print()
        if mapping_report is not None:
            print(mapping_estimation.format_report(mapping_report))
            print()
        print(format_checklist(verdict))
    return 0 if verdict["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
