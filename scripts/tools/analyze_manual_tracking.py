#!/usr/bin/env python3
"""Offline engineering measurements of real manual-input simulation records.

This tool reads records only. It publishes no ROS messages and changes no robot
configuration. A PASS means the declared local screening limits were met; it is
never physical-robot approval. Position is in metres and quaternions are xyzw.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np

AXES = ("X", "Y", "Z", "RX", "RY", "RZ")
LIMITS = {
    "moving_position_p95_mm": 5.0,
    "moving_orientation_p95_deg": 2.0,
    "hold_tail_position_max_mm": 1.0,
    "hold_tail_orientation_max_deg": 0.5,
    "idle_drift_mm": 0.1,
    "idle_drift_deg": 0.05,
    "stop_delay_s": 0.5,
    "stop_residual_mm": 1.0,
    "stop_residual_deg": 0.5,
    "target_age_s": 0.1,
    "feedback_gap_s": 0.2,
    "settling_window_s": 0.5,
    "stopped_window_s": 0.2,
    "stopped_linear_mm_s": 1.0,
    "stopped_angular_deg_s": 1.0,
    "stopped_joint_deg_s": 1.0,
}


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"n": 0, "rmse": None, "p50": None, "p95": None, "max": None}
    return {
        "n": len(values),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


def quaternion_angle_deg(left, right):
    """Geodesic rotation error, invariant to the quaternion double cover."""
    left, right = np.asarray(left, dtype=float), np.asarray(right, dtype=float)
    if left.shape[-1] != 4 or right.shape[-1] != 4:
        raise ValueError("expected xyzw quaternion")
    left = left / np.linalg.norm(left, axis=-1, keepdims=True)
    right = right / np.linalg.norm(right, axis=-1, keepdims=True)
    dot = np.clip(np.abs(np.sum(left * right, axis=-1)), 0.0, 1.0)
    return np.degrees(2.0 * np.arccos(dot))


def relative_rotvec_deg(reference, current):
    """current * inverse(reference), expressed in the reference base frame."""
    ref, cur = np.asarray(reference, dtype=float), np.asarray(current, dtype=float)
    ref, cur = ref / np.linalg.norm(ref), cur / np.linalg.norm(cur)
    rv, rw, cv, cw = -ref[:3], ref[3], cur[:3], cur[3]
    vector = cw * rv + rw * cv + np.cross(cv, rv)
    scalar = cw * rw - np.dot(cv, rv)
    if scalar < 0:
        vector, scalar = -vector, -scalar
    norm = float(np.linalg.norm(vector))
    return np.degrees(vector * (2.0 if norm < 1e-12 else 2.0 * math.atan2(norm, scalar) / norm))


def mean_quaternion(values):
    values = np.asarray(values, dtype=float)
    values = values / np.linalg.norm(values, axis=1, keepdims=True)
    _, vectors = np.linalg.eigh(values.T @ values)
    result = vectors[:, -1]
    return -result if np.dot(result, values[0]) < 0 else result


def _first_crossing(times, values, threshold):
    matches = np.flatnonzero(np.asarray(values) >= threshold)
    return None if not len(matches) else float(np.asarray(times)[matches[0]])


def axis_response(move, hold, joints, targets):
    """10% closed-loop response and hold overshoot, not a network delay."""
    axis = move.get("axis")
    if not isinstance(axis, int) or not 0 <= axis < 6:
        return None
    selected = [r for r in joints if move["host_s"] <= r["host_s"] < hold["end_s"]]
    reference_joints = [r for r in joints if r["host_s"] <= move["host_s"]]
    references = [r for r in targets if r["host_s"] <= move["host_s"]]
    accepted = [r for r in targets if move["host_s"] <= r["host_s"] < hold["end_s"]]
    if not selected or not reference_joints or not accepted:
        return None
    initial = references[-1] if references else accepted[0]
    terminal = accepted[-1]
    first_actual = reference_joints[-1]
    sign = float(move.get("axes", [1] * 6)[axis]) or 1.0
    if axis < 3:
        commanded_delta = (terminal["p"][axis] - initial["p"][axis]) * 1000 * sign
        target_progress = [(r["p"][axis] - initial["p"][axis]) * 1000 * sign for r in accepted]
        actual_progress = [(r["tcp_p"][axis] - first_actual["tcp_p"][axis]) * 1000 * sign for r in selected]
        overshoot = [
            (r["tcp_p"][axis] - terminal["p"][axis]) * 1000 * sign for r in selected if r["host_s"] >= hold["host_s"]
        ]
        unit = "mm"
    else:
        component = axis - 3
        commanded_delta = relative_rotvec_deg(initial["q"], terminal["q"])[component] * sign
        target_progress = [relative_rotvec_deg(initial["q"], r["q"])[component] * sign for r in accepted]
        actual_progress = [relative_rotvec_deg(first_actual["tcp_q"], r["tcp_q"])[component] * sign for r in selected]
        overshoot = [
            relative_rotvec_deg(terminal["q"], r["tcp_q"])[component] * sign
            for r in selected
            if r["host_s"] >= hold["host_s"]
        ]
        unit = "deg"
    threshold = abs(float(commanded_delta)) * 0.1
    t_target = (
        _first_crossing([r["stamp_s"] for r in accepted], target_progress, threshold) if threshold > 1e-6 else None
    )
    t_actual = (
        _first_crossing([r["stamp_s"] for r in selected], actual_progress, threshold) if threshold > 1e-6 else None
    )
    return {
        "unit": unit,
        "published_target_increment": float(commanded_delta),
        "target_10pct_stamp_s": t_target,
        "actual_10pct_stamp_s": t_actual,
        "closed_loop_10pct_delay_s": None if t_target is None or t_actual is None else t_actual - t_target,
        "hold_axis_overshoot": max(0.0, float(max(overshoot))) if overshoot else None,
    }


def align_targets(joints, targets, *, max_age_s=0.1):
    """Causal zero-order hold by source timestamps; no future target is used.

    Returned arrays preserve the receipt order of joints. Invalid/missing target
    entries are NaN and counted in coverage; they never lower an error metric.
    """
    count = len(joints)
    result = {
        "valid": np.zeros(count, dtype=bool),
        "age_s": np.full(count, np.nan),
        "position_mm": np.full(count, np.nan),
        "orientation_deg": np.full(count, np.nan),
        "target_p": np.full((count, 3), np.nan),
        "target_q": np.full((count, 4), np.nan),
    }
    if not count or not targets:
        return result
    target_stamps = np.array([r["stamp_s"] for r in targets], dtype=float)
    order = np.argsort(target_stamps, kind="stable")
    target_stamps = target_stamps[order]
    positions = np.array([targets[i]["p"] for i in order], dtype=float)
    rotations = np.array([targets[i]["q"] for i in order], dtype=float)
    stamps = np.array([r["stamp_s"] for r in joints], dtype=float)
    indices = np.searchsorted(target_stamps, stamps, side="right") - 1
    exists = indices >= 0
    safe = np.maximum(indices, 0)
    ages = stamps - target_stamps[safe]
    result["age_s"][exists] = ages[exists]
    finite_targets = np.isfinite(positions[safe]).all(axis=1) & np.isfinite(rotations[safe]).all(axis=1)
    finite_targets &= np.linalg.norm(rotations[safe], axis=1) > 1e-12
    valid = exists & (ages >= 0.0) & (ages <= max_age_s) & finite_targets
    actual_p = np.array([r["tcp_p"] for r in joints], dtype=float)
    actual_q = np.array([r["tcp_q"] for r in joints], dtype=float)
    valid &= np.isfinite(actual_p).all(axis=1) & np.isfinite(actual_q).all(axis=1)
    valid &= np.linalg.norm(actual_q, axis=1) > 1e-12
    result["valid"] = valid
    result["target_p"][valid] = positions[safe[valid]]
    result["target_q"][valid] = rotations[safe[valid]]
    result["position_mm"][valid] = np.linalg.norm(actual_p[valid] - positions[safe[valid]], axis=1) * 1000.0
    result["orientation_deg"][valid] = quaternion_angle_deg(actual_q[valid], rotations[safe[valid]])
    return result


def sustained_start(times, condition, duration_s, *, max_gap_s=0.05):
    """Beginning of a sampled condition sustained for duration; gaps break it."""
    times, condition = np.asarray(times, dtype=float), np.asarray(condition, dtype=bool)
    started = None
    previous = None
    for stamp, good in zip(times, condition):
        if not good or (previous is not None and stamp - previous > max_gap_s):
            started = None
        if good:
            if started is None:
                started = float(stamp)
            if stamp - started >= duration_s - 1e-9:
                return started
        previous = stamp
    return None


def _rows(records, kind):
    return [row for row in records if row.get("kind") == kind]


def _check(name, value, limit=None, *, expected=None):
    if value is None:
        passed = None
    elif expected is not None:
        passed = value == expected
    else:
        passed = value <= limit
    return {"name": name, "value": value, "limit": limit, "expected": expected, "passed": passed}


def _status(checks):
    return (
        "FAIL"
        if any(c["passed"] is False for c in checks)
        else ("INCOMPLETE" if any(c["passed"] is None for c in checks) else "PASS")
    )


def _finite_vectors(rows, fields):
    return sum(
        any(not np.isfinite(np.asarray(row.get(field, []), dtype=float)).all() for field in fields) for row in rows
    )


def valid_joint(row):
    try:
        return all(
            np.asarray(row[field], dtype=float).shape == (size,) and np.isfinite(row[field]).all()
            for field, size in (("q", 6), ("dq", 6), ("tcp_p", 3), ("tcp_q", 4))
        ) and (math.isfinite(float(row["stamp_s"])) and np.linalg.norm(row["tcp_q"]) > 1e-12)
    except (KeyError, TypeError, ValueError):
        return False


def _regressions(rows, field):
    if len(rows) < 2:
        return 0
    return int(np.count_nonzero(np.diff([row[field] for row in rows]) < -1e-9))


def _window_motion(joints):
    if not joints:
        return {"position_max_mm": None, "orientation_max_deg": None, "joint_max_deg": None, "duration_s": 0.0}
    p = np.array([r["tcp_p"] for r in joints], dtype=float)
    q = np.array([r["q"] for r in joints], dtype=float)
    return {
        "position_max_mm": float(np.max(np.linalg.norm(p - p[0], axis=1))) * 1000.0,
        "orientation_max_deg": float(np.max(quaternion_angle_deg(joints[0]["tcp_q"], [r["tcp_q"] for r in joints]))),
        "joint_max_deg": float(np.max(np.abs(q - q[0]))) * 180.0 / math.pi,
        "duration_s": float(joints[-1]["host_s"] - joints[0]["host_s"]),
    }


def phase_intervals(records, *, end_s=None):
    phases = _rows(records, "phase")
    if not records:
        return []
    end_s = max(row["host_s"] for row in records) if end_s is None else end_s
    return [
        {**phase, "end_s": phases[i + 1]["host_s"] if i + 1 < len(phases) else end_s} for i, phase in enumerate(phases)
    ]


def physical_clock_factor(records):
    """Compare clock progress, not clock-message counts, inside the measurement.

    Duplicate physical stamps keep their first receipt. One-second windows are
    disjoint host-time bins anchored at measurement_start; only bins with at
    least 0.95 s sampled host coverage contribute. This is descriptive and does
    not add an acceptance gate. Callback batching can affect individual windows.
    """
    clocks = _rows(records, "clock")
    if not clocks:
        return None
    starts, ends = _rows(records, "measurement_start"), _rows(records, "measurement_end")
    if not starts or not ends:
        return {"real_time_factor": None, "reason": "measurement_window_missing"}
    start, end = float(starts[0]["host_s"]), float(ends[-1]["host_s"])
    selected = [r for r in clocks if start <= r["host_s"] <= end and math.isfinite(r["stamp_s"])]
    if len(selected) < 2 or end <= start:
        return {"real_time_factor": None, "reason": "insufficient_clock_coverage"}
    regressions = _regressions(selected, "stamp_s")
    seen, unique = set(), []
    for row in selected:
        if row["stamp_s"] not in seen:
            seen.add(row["stamp_s"])
            unique.append(row)
    result = {
        "measurement_start_s": start,
        "measurement_end_s": end,
        "recorded_samples": len(selected),
        "unique_physical_stamps": len(unique),
        "duplicate_stamps": len(selected) - len(unique),
        "physical_time_regressions": regressions,
        "real_time_factor": None,
        "one_second_window_factor": None,
    }
    if regressions or len(unique) < 2:
        result["reason"] = "physical_time_regressed" if regressions else "insufficient_unique_clock_stamps"
        return result
    host = np.array([row["host_s"] for row in unique])
    physical = np.array([row["stamp_s"] for row in unique])
    host_delta, physical_delta = float(host[-1] - host[0]), float(physical[-1] - physical[0])
    if host_delta <= 0:
        result["reason"] = "invalid_host_interval"
        return result
    result.update(
        {
            "covered_host_start_s": float(host[0]),
            "covered_host_end_s": float(host[-1]),
            "host_duration_s": host_delta,
            "physical_duration_s": physical_delta,
            "real_time_factor": physical_delta / host_delta,
        }
    )
    windows = []
    for index in range(int(math.floor(end - start))):
        mask = (host >= start + index) & (host < start + index + 1.0)
        selected_ids = np.flatnonzero(mask)
        if len(selected_ids) < 2:
            continue
        first, last = selected_ids[0], selected_ids[-1]
        span = float(host[last] - host[first])
        if span >= 0.95:
            windows.append(
                {
                    "host_start_s": start + index,
                    "sampled_host_span_s": span,
                    "factor": float((physical[last] - physical[first]) / span),
                }
            )
    values = np.array([row["factor"] for row in windows])
    result["one_second_windows"] = windows
    result["one_second_window_factor"] = {
        "n": len(values),
        "min": float(np.min(values)) if len(values) else None,
        "p05": float(np.percentile(values, 5)) if len(values) else None,
        "p50": float(np.percentile(values, 50)) if len(values) else None,
        "p95": float(np.percentile(values, 95)) if len(values) else None,
        "max": float(np.max(values)) if len(values) else None,
    }
    return result


def analyze_run(run_dir):
    run_dir = Path(run_dir)
    meta = json.loads((run_dir / "meta.json").read_text(encoding="utf-8"))
    records = []
    malformed = []
    with (run_dir / "raw.jsonl").open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
                if not isinstance(row, dict) or not math.isfinite(float(row["host_s"])):
                    raise ValueError("record/host timestamp invalid")
                records.append(row)
            except (ValueError, TypeError, KeyError) as error:
                malformed.append({"line": number, "error": str(error)})
    all_joints, targets, internal = _rows(records, "joint"), _rows(records, "target"), _rows(records, "internal_target")
    joints = [row for row in all_joints if valid_joint(row)]
    controllers = _rows(records, "controller")
    phases = phase_intervals(records)
    aligned = align_targets(joints, targets, max_age_s=LIMITS["target_age_s"])
    bounded = align_targets(joints, internal, max_age_s=LIMITS["target_age_s"])
    host = np.array([row["host_s"] for row in joints])
    stamp = np.array([row["stamp_s"] for row in joints])
    q = np.array([row["q"] for row in joints], dtype=float).reshape((-1, 6))
    dq = np.array([row["dq"] for row in joints], dtype=float).reshape((-1, 6))
    p = np.array([row["tcp_p"] for row in joints], dtype=float).reshape((-1, 3))
    orientations = np.array([row["tcp_q"] for row in joints], dtype=float).reshape((-1, 4))
    dt = np.diff(stamp)
    linear_speed = np.full(len(joints), np.nan)
    angular_speed = np.full(len(joints), np.nan)
    if len(joints) > 1:
        good_dt = dt > 1e-9
        linear_speed[1:][good_dt] = np.linalg.norm(np.diff(p, axis=0)[good_dt], axis=1) * 1000.0 / dt[good_dt]
        angular_speed[1:][good_dt] = (
            quaternion_angle_deg(orientations[:-1][good_dt], orientations[1:][good_dt]) / dt[good_dt]
        )
    config = meta.get("config", {})
    model = config.get("model", meta.get("model", {}))
    speeds = model.get("joint_velocity_rad_s")
    lower, upper = model.get("joint_lower_rad"), model.get("joint_upper_rad")
    limit_violations = (
        None
        if lower is None or upper is None
        else int(np.count_nonzero(np.any((q < np.asarray(lower) - 1e-9) | (q > np.asarray(upper) + 1e-9), axis=1)))
    )
    velocity_ratio = None if speeds is None or not len(dq) else float(np.max(np.abs(dq) / np.asarray(speeds)))
    joint_receipt_age = np.array([row.get("receipt_ros_s", row["stamp_s"]) - row["stamp_s"] for row in joints])
    clock_rows = _rows(records, "clock")
    counts = Counter(row.get("kind", "unknown") for row in records)
    servo_counts = Counter(int(row["code"]) for row in _rows(records, "servo"))
    reason_counts = {}
    for kind, key in (("input", "reason"), ("tracking", "state")):
        reason_counts[kind] = dict(
            Counter(str(row.get("value", {}).get(key, "unknown")) for row in _rows(records, kind))
        )
    tracking_faults = sum(bool(row.get("value", {}).get("restart_required")) for row in _rows(records, "tracking"))
    hard_input_reasons = {"feedback_time_regressed", "feedback_future_stamp", "invalid_quaternion_norm", "invalid_pose"}
    input_faults = sum(row.get("value", {}).get("reason") in hard_input_reasons for row in _rows(records, "input"))
    checks = [
        _check("completed_protocol", meta.get("completed"), expected=True),
        _check("malformed_records", len(malformed), 0),
        _check("invalid_joint_records", len(all_joints) - len(joints), 0),
        _check("nonfinite_target_records", _finite_vectors(targets + internal, ("p", "q", "stamp_s")), 0),
        _check(
            "nonfinite_controller_records",
            _finite_vectors(controllers, ("desired_q", "actual_q", "error_q", "stamp_s")),
            0,
        ),
        _check("feedback_time_regressions", _regressions(joints, "stamp_s"), 0),
        _check("target_time_regressions", _regressions(targets, "stamp_s"), 0),
        _check("physical_time_regressions", _regressions(clock_rows, "stamp_s"), 0),
        _check("feedback_source_max_gap_s", float(np.max(dt)) if len(dt) else None, LIMITS["feedback_gap_s"]),
        _check(
            "feedback_receipt_max_gap_s",
            float(np.max(np.diff(host))) if len(host) > 1 else None,
            LIMITS["feedback_gap_s"],
        ),
        _check("joint_limit_violations", limit_violations, 0),
        _check("joint_speed_limit_ratio", velocity_ratio, 1.0 + 1e-6),
        _check("tracking_restart_faults", tracking_faults, 0),
        _check("input_integrity_faults", input_faults, 0),
    ]
    phase_results, repeat_endpoints = [], []
    moving_mask = np.zeros(len(joints), dtype=bool)
    tracking_mask = np.zeros(len(joints), dtype=bool)
    controller_hosts = np.array([row["host_s"] for row in controllers])
    controller_moving = np.zeros(len(controllers), dtype=bool)
    controller_tracking = np.zeros(len(controllers), dtype=bool)
    unexpected_disabled = []
    for index, phase in enumerate(phases):
        start, end = phase["host_s"], phase["end_s"]
        mask = (host >= start) & (host < end)
        selected = np.flatnonzero(mask)
        category = phase.get("category")
        axis = phase.get("axis")
        row = {
            "name": phase["name"],
            "category": category,
            "axis": axis,
            "cycle": phase.get("cycle"),
            "duration_s": float(end - start),
            "sample_count": len(selected),
        }
        if category in {"move", "hold"}:
            tracking_mask |= mask
            controller_tracking |= (controller_hosts >= start) & (controller_hosts < end)
            if category == "move":
                moving_mask |= mask
                controller_moving |= (controller_hosts >= start) & (controller_hosts < end)
            # After the bounded cross-topic/phase transition interval, a lost
            # enable cannot count as accurate zero-error tracking.
            disabled = [
                status
                for status in _rows(records, "input")
                if start + 0.1 <= status["host_s"] < end and status.get("value", {}).get("enabled") is not True
            ]
            row["unexpected_disabled_samples"] = len(disabled)
            unexpected_disabled.extend(
                {"phase": phase["name"], "host_s": status["host_s"], "reason": status.get("value", {}).get("reason")}
                for status in disabled
            )
            row["target_coverage"] = float(np.mean(aligned["valid"][mask])) if len(selected) else 0.0
            row["position_mm"], row["orientation_deg"] = stats(aligned["position_mm"][mask]), stats(
                aligned["orientation_deg"][mask]
            )
            row["internal_target_coverage"] = float(np.mean(bounded["valid"][mask])) if len(selected) else 0.0
            row["internal_position_mm"], row["internal_orientation_deg"] = stats(bounded["position_mm"][mask]), stats(
                bounded["orientation_deg"][mask]
            )
            # Any missing alignment is visible and prevents a full pass.
            checks.append(_check(phase["name"] + ":target_coverage", row["target_coverage"], expected=1.0))
            if category == "hold":
                tail = mask & (host >= end - 1.0)
                row["tail_position_mm"] = stats(aligned["position_mm"][tail])
                row["tail_orientation_deg"] = stats(aligned["orientation_deg"][tail])
                tail_ids = np.flatnonzero(tail)
                row["tail_motion"] = _window_motion([joints[i] for i in tail_ids])
                row["tail_linear_speed_mm_s"] = stats(linear_speed[tail])
                row["tail_angular_speed_deg_s"] = stats(angular_speed[tail])
                if len(tail_ids):
                    row["tail_position_component_ptp_mm"] = (np.ptp(p[tail], axis=0) * 1000).tolist()
                    relative_angles = np.array(
                        [relative_rotvec_deg(orientations[tail_ids[0]], v) for v in orientations[tail]]
                    )
                    row["tail_rotation_component_ptp_deg"] = np.ptp(relative_angles, axis=0).tolist()
                    finite_tail = tail_ids[aligned["valid"][tail_ids]]
                    row["tail_position_error_change_mm"] = (
                        float(aligned["position_mm"][finite_tail[-1]] - aligned["position_mm"][finite_tail[0]])
                        if len(finite_tail)
                        else None
                    )
                    row["tail_orientation_error_change_deg"] = (
                        float(aligned["orientation_deg"][finite_tail[-1]] - aligned["orientation_deg"][finite_tail[0]])
                        if len(finite_tail)
                        else None
                    )
                within = (
                    aligned["valid"][mask]
                    & (aligned["position_mm"][mask] <= 1.0)
                    & (aligned["orientation_deg"][mask] <= 0.5)
                )
                settled = sustained_start(host[mask], within, LIMITS["settling_window_s"])
                row["settling_time_s"] = None if settled is None else settled - start
                row["initial_position_mm"] = stats(aligned["position_mm"][mask & (host < start + 0.1)])
                row["initial_orientation_deg"] = stats(aligned["orientation_deg"][mask & (host < start + 0.1)])
                checks.extend(
                    [
                        _check(phase["name"] + ":hold_tail_position_max_mm", row["tail_position_mm"]["max"], 1.0),
                        _check(
                            phase["name"] + ":hold_tail_orientation_max_deg", row["tail_orientation_deg"]["max"], 0.5
                        ),
                    ]
                )
                previous = phases[index - 1] if index else None
                if previous is not None and previous.get("category") == "move" and np.any(tail):
                    row["axis_response"] = axis_response(previous, phase, joints, targets)
                    before = np.flatnonzero(host <= previous["host_s"])
                    if len(before):
                        anchor = before[-1]
                        endpoint = np.mean(p[tail], axis=0) - p[anchor]
                        tail_q = mean_quaternion(orientations[tail])
                        accepted_targets = [r for r in targets if r["host_s"] < end]
                        final_target = accepted_targets[-1] if accepted_targets else None
                        repeat_endpoints.append(
                            {
                                "phase": phase["name"],
                                "axis": axis,
                                "position_delta_mm": (endpoint * 1000).tolist(),
                                "rotation_delta_deg": relative_rotvec_deg(orientations[anchor], tail_q).tolist(),
                                "target_delta_mm": (
                                    None
                                    if final_target is None
                                    else ((np.array(final_target["p"]) - p[anchor]) * 1000).tolist()
                                ),
                                "position_following_error_mm": (
                                    None
                                    if final_target is None
                                    else ((np.mean(p[tail], axis=0) - final_target["p"]) * 1000).tolist()
                                ),
                                "rotation_following_error_deg": (
                                    None
                                    if final_target is None
                                    else relative_rotvec_deg(final_target["q"], tail_q).tolist()
                                ),
                            }
                        )
        if category in {"release", "dropout"}:
            # Always retain release diagnostics; only dedicated stops have gates.
            before = np.flatnonzero(host <= start)
            extended = ([joints[before[-1]]] if len(before) else []) + [joints[i] for i in selected]
            row["residual_motion"] = _window_motion(extended)
            stable = np.zeros(len(joints), dtype=bool)
            if len(joints):
                stable = (linear_speed <= LIMITS["stopped_linear_mm_s"]) & (
                    angular_speed <= LIMITS["stopped_angular_deg_s"]
                )
                stable &= np.max(np.abs(dq), axis=1) <= math.radians(LIMITS["stopped_joint_deg_s"])
            stopped = sustained_start(host[mask], stable[mask], LIMITS["stopped_window_s"])
            row["stop_delay_s"] = None if stopped is None else stopped - start
            row["tail_drift"] = _window_motion([joints[i] for i in np.flatnonzero(mask & (host >= end - 0.5))])
            if "stop" in phase["name"]:
                checks.extend(
                    [
                        _check(phase["name"] + ":stop_delay_s", row["stop_delay_s"], 0.5),
                        _check(phase["name"] + ":residual_mm", row["residual_motion"]["position_max_mm"], 1.0),
                        _check(phase["name"] + ":residual_deg", row["residual_motion"]["orientation_max_deg"], 0.5),
                    ]
                )
        if category == "idle":
            row["drift"] = _window_motion([joints[i] for i in selected])
            if phase["name"] == "idle_end":
                checks.extend(
                    [
                        _check("idle_end:drift_mm", row["drift"]["position_max_mm"], 0.1),
                        _check("idle_end:drift_deg", row["drift"]["orientation_max_deg"], 0.05),
                        _check("idle_end:duration_min_s", row["drift"]["duration_s"] >= 9.8, expected=True),
                    ]
                )
        phase_results.append(row)
    moving_p, moving_r = stats(aligned["position_mm"][moving_mask]), stats(aligned["orientation_deg"][moving_mask])
    checks.extend(
        [
            _check("moving_position_p95_mm", moving_p["p95"], LIMITS["moving_position_p95_mm"]),
            _check("moving_orientation_p95_deg", moving_r["p95"], LIMITS["moving_orientation_p95_deg"]),
            _check("unexpected_disabled_during_tracking", len(unexpected_disabled), 0),
        ]
    )
    jtc_error = np.abs(np.array([r["error_q"] for r in controllers], dtype=float))
    trajectories = _rows(records, "trajectory")
    trajectory_period = np.diff([r["host_s"] for r in trajectories])
    trajectory_horizons, trajectory_gaps = [], []
    for trajectory in trajectories:
        points = trajectory.get("points", [])
        if not points:
            continue
        trajectory_horizons.extend(float(point["time_s"]) * 1000 for point in points if "time_s" in point)
        previous_q = np.searchsorted(host, trajectory["host_s"], side="right") - 1
        if previous_q >= 0 and trajectory["host_s"] - host[previous_q] <= 0.1:
            point_q = np.asarray(points[-1].get("q", []), dtype=float)
            names = trajectory.get("names", [])
            if len(names) == 6 and point_q.shape == (6,) and set(names) == {f"joint{i}" for i in range(1, 7)}:
                ordered = point_q[[names.index(f"joint{i}") for i in range(1, 7)]]
                trajectory_gaps.extend(np.abs(ordered - q[previous_q]) * 180 / math.pi)
    max_clamp_count = max(
        (row.get("value", {}).get("clamped_targets", 0) for row in _rows(records, "tracking")), default=0
    )
    closures = []
    for back_index, phase in enumerate(phases):
        if not phase["name"].endswith("back_hold"):
            continue
        out_name = phase["name"].replace("back_hold", "out_move")
        out = next((candidate for candidate in phases[:back_index] if candidate["name"] == out_name), None)
        if out is None:
            continue
        origin_ids = np.flatnonzero(host <= out["host_s"])
        final_ids = np.flatnonzero((host >= phase["end_s"] - 1.0) & (host < phase["end_s"]))
        initial_targets = [r for r in targets if r["host_s"] <= out["host_s"]]
        final_targets = [r for r in targets if r["host_s"] < phase["end_s"]]
        if not initial_targets:
            initial_targets = [r for r in targets if out["host_s"] <= r["host_s"] < out["end_s"]][:1]
        if not len(origin_ids) or not len(final_ids) or not initial_targets or not final_targets:
            continue
        initial_target, final_target = initial_targets[-1], final_targets[-1]
        initial_actual = joints[origin_ids[-1]]
        final_actual_p, final_actual_q = np.mean(p[final_ids], axis=0), mean_quaternion(orientations[final_ids])
        desired_closure = (np.array(final_target["p"]) - initial_target["p"]) * 1000
        actual_return = (final_actual_p - initial_actual["tcp_p"]) * 1000
        final_error = (final_actual_p - final_target["p"]) * 1000
        closures.append(
            {
                "phase": phase["name"],
                "axis": phase.get("axis"),
                "target_closure_vector_mm": desired_closure.tolist(),
                "target_closure_mm": float(np.linalg.norm(desired_closure)),
                "target_closure_deg": float(quaternion_angle_deg(initial_target["q"], final_target["q"])),
                "actual_return_vector_mm": actual_return.tolist(),
                "actual_return_mm": float(np.linalg.norm(actual_return)),
                "actual_return_deg": float(quaternion_angle_deg(initial_actual["tcp_q"], final_actual_q)),
                "actual_vs_final_target_vector_mm": final_error.tolist(),
                "actual_vs_final_target_mm": float(np.linalg.norm(final_error)),
                "actual_vs_final_target_deg": float(quaternion_angle_deg(final_target["q"], final_actual_q)),
            }
        )
    result = {
        "run_id": meta.get("run_id", run_dir.name),
        "simulator": meta.get("simulator"),
        "protocol": meta.get("protocol"),
        "gui": meta.get("gui"),
        "measurement_sha256": meta.get("measurement_sha256"),
        "source_sha": meta.get("source_sha"),
        "config_sha256": meta.get("config_sha256"),
        "controllers_sha256": meta.get("controllers_sha256"),
        "description_sha256": meta.get("description_sha256"),
        "directory": str(run_dir),
        "status": _status(checks),
        "completed": meta.get("completed"),
        "failure": meta.get("failure"),
        "sample_counts": dict(counts),
        "physical_clock": physical_clock_factor(records),
        "malformed": malformed,
        "duration_s": float(max(host) - min(host)) if len(host) else 0.0,
        "checks": checks,
        "phases": phase_results,
        "repeat_endpoints": repeat_endpoints,
        "return_closure": closures,
        "moving_position_mm": moving_p,
        "moving_orientation_deg": moving_r,
        "internal_moving_position_mm": stats(bounded["position_mm"][moving_mask]),
        "internal_moving_orientation_deg": stats(bounded["orientation_deg"][moving_mask]),
        "tracking_alignment_coverage": (
            float(np.mean(aligned["valid"][tracking_mask])) if np.any(tracking_mask) else 0.0
        ),
        "internal_alignment_coverage": (
            float(np.mean(bounded["valid"][tracking_mask])) if np.any(tracking_mask) else 0.0
        ),
        "joint_feedback_period_ms": stats(dt[dt > 0] * 1000),
        "feedback_receipt_delay_ms": stats(joint_receipt_age * 1000),
        "negative_feedback_receipt_age_count": int(np.count_nonzero(joint_receipt_age < -0.02)),
        "servo_code_counts": dict(sorted(servo_counts.items())),
        "reason_counts": reason_counts,
        "unexpected_disabled_during_tracking": unexpected_disabled,
        "clamped_targets": int(max_clamp_count),
        "joint_speed_limit_ratio": velocity_ratio,
        "jtc_absolute_error_deg": stats(jtc_error.ravel() * 180 / math.pi),
        "jtc_moving_error_deg": stats(jtc_error[controller_moving].ravel() * 180 / math.pi),
        "jtc_tracking_error_deg": stats(jtc_error[controller_tracking].ravel() * 180 / math.pi),
        "trajectory_publication_period_ms": stats(trajectory_period * 1000),
        "trajectory_point_horizon_ms": stats(trajectory_horizons),
        "raw_trajectory_reference_gap_deg": stats(trajectory_gaps),
        "jtc_per_joint_error_deg": (
            [stats(jtc_error[:, i] * 180 / math.pi) for i in range(6)] if len(controllers) else []
        ),
    }
    series = {
        "host": host,
        "stamp": stamp,
        "p": p,
        "q": orientations,
        "aligned": aligned,
        "bounded": bounded,
        "linear_speed": linear_speed,
        "angular_speed": angular_speed,
        "phases": phases,
        "servo": _rows(records, "servo"),
        "joints": joints,
    }
    return result, series


def group_identity(run):
    fields = (
        "simulator",
        "protocol",
        "gui",
        "measurement_sha256",
        "source_sha",
        "config_sha256",
        "controllers_sha256",
        "description_sha256",
    )
    return {key: run.get(key) for key in fields}


def group_id(run):
    return hashlib.sha256(json.dumps(group_identity(run), sort_keys=True).encode()).hexdigest()[:12]


def repeatability(runs):
    groups = defaultdict(list)
    for run in runs:
        if run.get("completed") is not True:
            continue
        for row in run["repeat_endpoints"]:
            groups[(group_id(run), row["phase"])].append({"run": run["run_id"], "identity": group_identity(run), **row})
    output = []
    for (identity, phase), rows in sorted(groups.items(), key=lambda pair: str(pair[0])):
        pos = np.array([row["position_delta_mm"] for row in rows])
        rot = np.array([row["rotation_delta_deg"] for row in rows])
        output.append(
            {
                "group_id": identity,
                **rows[0]["identity"],
                "phase": phase,
                "n": len(rows),
                "runs": [row["run"] for row in rows],
                "position_component_std_mm": np.std(pos, axis=0, ddof=1).tolist() if len(rows) > 1 else None,
                "position_component_range_mm": np.ptp(pos, axis=0).tolist(),
                "rotation_component_std_deg": np.std(rot, axis=0, ddof=1).tolist() if len(rows) > 1 else None,
                "rotation_component_range_deg": np.ptp(rot, axis=0).tolist(),
                "position_deltas_mm": pos.tolist(),
                "rotation_deltas_deg": rot.tolist(),
                "position_following_error_component_std_mm": (
                    np.std([row["position_following_error_mm"] for row in rows], axis=0, ddof=1).tolist()
                    if len(rows) > 1 and all(row.get("position_following_error_mm") is not None for row in rows)
                    else None
                ),
                "target_delta_component_std_mm": (
                    np.std([row["target_delta_mm"] for row in rows], axis=0, ddof=1).tolist()
                    if len(rows) > 1 and all(row.get("target_delta_mm") is not None for row in rows)
                    else None
                ),
            }
        )
    return output


def write_plots(result, series, output):
    cache = Path(__file__).resolve().parents[2] / "build/cache/matplotlib"
    cache.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache))
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not len(series["host"]):
        return []
    base = float(series["host"][0])
    t = series["host"] - base
    target, actual = series["aligned"], series["p"]
    fig, axes = plt.subplots(4, 1, figsize=(13.5, 10), sharex=True, layout="constrained")
    colors = ("#176a8b", "#c06b26", "#426e44")
    for axis in range(3):
        baseline = actual[0, axis]
        axes[0].plot(t, (actual[:, axis] - baseline) * 1000, color=colors[axis], lw=1.1, label=AXES[axis] + " actual")
        axes[0].plot(
            t,
            (target["target_p"][:, axis] - baseline) * 1000,
            color=colors[axis],
            lw=0.9,
            ls="--",
            label=AXES[axis] + " target",
        )
    # Quaternion logs below are only for displaying small relative rotations;
    # numeric errors above always use the geodesic quaternion angle.
    ref = series["q"][0]
    actual_r = np.array([relative_rotvec_deg(ref, q) for q in series["q"]])
    target_r = np.full_like(actual_r, np.nan)
    for i in np.flatnonzero(target["valid"]):
        target_r[i] = relative_rotvec_deg(ref, target["target_q"][i])
    for axis in range(3):
        axes[1].plot(t, actual_r[:, axis], color=colors[axis], lw=1.1, label=AXES[axis + 3] + " actual")
        axes[1].plot(t, target_r[:, axis], color=colors[axis], lw=0.9, ls="--", label=AXES[axis + 3] + " target")
    axes[2].plot(t, target["position_mm"], color="#176a8b", label="Raw target position error / mm")
    axes[2].plot(t, target["orientation_deg"], color="#b34a3c", label="Raw target rotation error / deg")
    if series["bounded"]["valid"].any():
        axes[2].plot(
            t,
            series["bounded"]["position_mm"],
            color="#426e44",
            lw=0.8,
            ls="--",
            label="Clamped target position error / mm",
        )
    servos = series["servo"]
    if servos:
        axes[3].step(
            [r["host_s"] - base for r in servos],
            [r["code"] for r in servos],
            where="post",
            color="#655381",
            label="Servo status code",
        )
    for phase in series["phases"]:
        begin, end = phase["host_s"] - base, phase["end_s"] - base
        if phase.get("category") == "hold":
            for ax in axes:
                ax.axvspan(begin, end, color="#e9edf0", alpha=0.35, linewidth=0)
        if phase.get("category") == "move":
            axes[0].text(
                begin,
                1.01,
                phase["name"],
                rotation=60,
                ha="left",
                va="bottom",
                fontsize=6,
                transform=axes[0].get_xaxis_transform(),
            )
    for ax in axes:
        ax.grid(axis="y", color="#d8dce0", lw=0.5)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(loc="upper right", ncol=3, fontsize=7)
    axes[0].set_ylabel("Position delta / mm")
    axes[1].set_ylabel("Rotation delta / deg")
    axes[2].set_ylabel("Geodesic / Euclidean error")
    axes[3].set_ylabel("Status code")
    axes[3].set_xlabel("Host monotonic elapsed / s; shaded intervals keep Grip held with zero direction")
    fig.suptitle(
        f"{result['run_id']} | {result['simulator']} | {result['status']} | source-timestamp causal alignment",
        fontsize=12,
    )
    stem = output / (result["run_id"] + "_tracking")
    fig.savefig(stem.with_suffix(".png"), dpi=170)
    fig.savefig(stem.with_suffix(".svg"))
    plt.close(fig)
    paths = [str(stem.with_suffix(".png")), str(stem.with_suffix(".svg"))]
    focus = []
    for focus_axis in (0, 3):
        for i, phase in enumerate(series["phases"][:-1]):
            following = series["phases"][i + 1]
            if (
                phase.get("axis") == focus_axis
                and phase.get("category") == "move"
                and following.get("category") == "hold"
            ):
                focus.append((phase, following))
                break
    if focus:
        fig, axes = plt.subplots(2, len(focus), figsize=(6.0 * len(focus), 6.0), squeeze=False, layout="constrained")
        for column, (move, hold) in enumerate(focus):
            axis = move["axis"]
            start, end = move["host_s"], hold["end_s"]
            mask = (series["host"] >= start - 0.1) & (series["host"] < end)
            anchor = max(0, np.searchsorted(series["host"], start, side="right") - 1)
            local_t = series["host"][mask] - start
            if axis < 3:
                actual_value = (actual[mask, axis] - actual[anchor, axis]) * 1000
                target_value = (target["target_p"][mask, axis] - actual[anchor, axis]) * 1000
                errors, unit, limit = target["position_mm"][mask], "mm", 1.0
            else:
                reference = series["q"][anchor]
                actual_value = [relative_rotvec_deg(reference, q)[axis - 3] for q in series["q"][mask]]
                target_value = np.full(np.count_nonzero(mask), np.nan)
                for j, target_index in enumerate(np.flatnonzero(mask)):
                    if target["valid"][target_index]:
                        target_value[j] = relative_rotvec_deg(reference, target["target_q"][target_index])[axis - 3]
                errors, unit, limit = target["orientation_deg"][mask], "deg", 0.5
            axes[0, column].plot(local_t, target_value, color="#176a8b", ls="--", label="Published target")
            axes[0, column].plot(local_t, actual_value, color="#b34a3c", label="Measured actual TCP")
            axes[0, column].set_ylabel(f"{AXES[axis]} increment / {unit}")
            axes[0, column].set_title(
                f"{AXES[axis]} out: {move['end_s'] - start:.2f}s move + {end - hold['host_s']:.2f}s hold"
            )
            axes[1, column].plot(local_t, errors, color="#176a8b", label="Pose error")
            axes[1, column].axhline(limit, color="#b34a3c", ls=":", label=f"Static screening {limit:g} {unit}")
            axes[1, column].set_ylabel(f"{'Euclidean position' if axis < 3 else 'Geodesic rotation'} error / {unit}")
            for ax in axes[:, column]:
                ax.axvspan(hold["host_s"] - start, end - start, color="#e9edf0", alpha=0.6, linewidth=0)
                ax.axvline(hold["host_s"] - start, color="#777777", lw=0.8, ls="--")
                ax.set_xlabel("Time from direction press / s; shaded: direction=0, Grip remains held")
                ax.grid(axis="y", color="#d8dce0", lw=0.5)
                ax.legend(fontsize=8)
                ax.spines[["top", "right"]].set_visible(False)
        fig.suptitle(f"{result['run_id']} | {result['simulator']} | closed-loop response detail", fontsize=12)
        focus_stem = output / (result["run_id"] + "_focus")
        for suffix in (".png", ".svg"):
            fig.savefig(focus_stem.with_suffix(suffix), dpi=170)
            paths.append(str(focus_stem.with_suffix(suffix)))
        plt.close(fig)
    stop_phases = [p for p in series["phases"] if p.get("category") in {"release", "dropout"} and "stop" in p["name"]]
    if stop_phases:
        fig, axes = plt.subplots(
            len(stop_phases), 2, figsize=(11, 3.1 * len(stop_phases)), squeeze=False, layout="constrained"
        )
        for row_index, phase in enumerate(stop_phases):
            mask = (series["host"] >= phase["host_s"] - 0.2) & (series["host"] < phase["end_s"])
            local_t = series["host"][mask] - phase["host_s"]
            axes[row_index, 0].plot(local_t, series["linear_speed"][mask], label="TCP linear / mm/s")
            axes[row_index, 0].plot(local_t, series["angular_speed"][mask], label="TCP angular / deg/s")
            selected = np.flatnonzero(mask)
            first = np.flatnonzero(series["host"] <= phase["host_s"])
            if len(first) and len(selected):
                anchor = first[-1]
                axes[row_index, 1].plot(
                    local_t,
                    np.linalg.norm(actual[mask] - actual[anchor], axis=1) * 1000,
                    label="Position from stop / mm",
                )
                axes[row_index, 1].plot(
                    local_t,
                    quaternion_angle_deg(series["q"][anchor], series["q"][mask]),
                    label="Rotation from stop / deg",
                )
            for ax in axes[row_index]:
                ax.axvline(0, color="#b34a3c", ls="--", lw=0.8)
                ax.set_title(phase["name"])
                ax.set_xlabel("Time from input stop / s")
                ax.grid(axis="y", color="#d8dce0", lw=0.5)
                ax.legend(fontsize=8)
                ax.spines[["top", "right"]].set_visible(False)
        stop_stem = output / (result["run_id"] + "_stopping")
        for suffix in (".png", ".svg"):
            fig.savefig(stop_stem.with_suffix(suffix), dpi=170)
            paths.append(str(stop_stem.with_suffix(suffix)))
        plt.close(fig)
    return paths


def markdown_report(summary):
    lines = [
        "# 无手柄仿真跟踪测量",
        "",
        f"工程筛查结果：**{summary['status']}**。门槛在测量前固定，仿真结果不构成真机执行批准。",
        "",
        "实际TCP由收到的实测关节和同一模型只读FK得到。目标按源时间戳作因果零阶对齐；超过100ms的目标被标为缺失，不参与误差分位数。保持段持续使能但方向为零；松开Grip停止段单独测量。",
        "",
        "| 运行 | 引擎 | 动态位置 P95 / mm | 动态姿态 P95 / ° | 数据覆盖率 | 判定 |",
        "|---|---|---:|---:|---:|---|",
    ]
    fmt = lambda value, digits=3: "—" if value is None else f"{value:.{digits}f}"
    for run in summary["runs"]:
        lines.append(
            f"| {run['run_id']} | {run['simulator']} | {fmt(run['moving_position_mm']['p95'])} | {fmt(run['moving_orientation_deg']['p95'])} | {run['tracking_alignment_coverage']:.2%} | {run['status']} |"
        )
    if summary.get("groups"):
        lines.extend(
            [
                "",
                "协议、显示模式、采集脚本和部署哈希一致才合并；未完成的尝试保留判定，但不加入重复性。",
                "",
                "| 对照组 | 引擎 / 协议 / GUI | 尝试 / 完成数 | 测量脚本SHA前12位 | 判定 |",
                "|---|---|---:|---|---|",
            ]
        )
        for group in summary["groups"]:
            lines.append(
                f"| {group['group_id']} | {group['simulator']} / {group['protocol']} / {group['gui']} | {group['attempt_count']} / {group['completed_count']} | {(group['measurement_sha256'] or 'unknown')[:12]} | {group['status']} |"
            )
    for run in summary["runs"]:
        lines.extend(
            [
                "",
                f"## {run['run_id']}",
                "",
                f"记录时长 {run['duration_s']:.2f}s，joint={run['sample_counts'].get('joint', 0)}，target={run['sample_counts'].get('target', 0)}；Servo码分布 `{run['servo_code_counts']}`。",
                "",
                "| 段 | 全段位置 P95 / mm | 全段姿态 P95 / ° | 保持末秒位置 max / mm | 保持末秒姿态 max / ° | 同时到位 / s |",
                "|---|---:|---:|---:|---:|---:|",
            ]
        )
        for phase in run["phases"]:
            if phase["category"] in {"move", "hold"}:
                settling = fmt(phase.get("settling_time_s"))
                if phase["category"] == "hold" and phase.get("settling_time_s") is None:
                    settling = f"未在{phase['duration_s']:.2f}s内到位"
                lines.append(
                    f"| {phase['name']} | {fmt(phase['position_mm']['p95'])} | {fmt(phase['orientation_deg']['p95'])} | {fmt(phase.get('tail_position_mm', {}).get('max'))} | {fmt(phase.get('tail_orientation_deg', {}).get('max'))} | {settling} |"
                )
        lines.extend(
            [
                "",
                "保持尾部实际运动与响应：10%延迟指闭环位姿响应，包含控制与执行，不称为网络延迟。",
                "",
                "| 保持段 | 10%响应延迟 / ms | 沿轴超调 / mm或° | 尾部位置变化 / mm | 尾部姿态变化 / ° | 尾部最大线/角速度 / mm·s⁻¹, °·s⁻¹ |",
                "|---|---:|---:|---:|---:|---|",
            ]
        )
        for phase in run["phases"]:
            if phase["category"] != "hold":
                continue
            response, motion = phase.get("axis_response") or {}, phase.get("tail_motion", {})
            delay = response.get("closed_loop_10pct_delay_s")
            lines.append(
                f"| {phase['name']} | {fmt(None if delay is None else delay * 1000)} | {fmt(response.get('hold_axis_overshoot'))} {response.get('unit', '')} | {fmt(motion.get('position_max_mm'))} | {fmt(motion.get('orientation_max_deg'))} | {fmt(phase['tail_linear_speed_mm_s']['max'])}, {fmt(phase['tail_angular_speed_deg_s']['max'])} |"
            )
        if run["return_closure"]:
            lines.extend(
                [
                    "",
                    "回程分解：目标积分闭合、实际相对起点、实际相对最终目标分别统计，不能混称回位误差。",
                    "",
                    "| 回程保持段 | 目标闭合 / mm, ° | 实际回到起点 / mm, ° | 实际相对最终目标 / mm, ° |",
                    "|---|---|---|---|",
                ]
            )
            for closure in run["return_closure"]:
                lines.append(
                    f"| {closure['phase']} | {fmt(closure['target_closure_mm'])}, {fmt(closure['target_closure_deg'])} | {fmt(closure['actual_return_mm'])}, {fmt(closure['actual_return_deg'])} | {fmt(closure['actual_vs_final_target_mm'])}, {fmt(closure['actual_vs_final_target_deg'])} |"
                )
        failures = [c for c in run["checks"] if c["passed"] is not True]
        lines.extend(["", "未满足或未完成的检查：", ""])
        lines.extend(
            [f"- {c['name']}: {c['value']}; limit={c['limit']}, expected={c['expected']}" for c in failures]
            or ["- 无。"]
        )
        lines.extend(["", "停止与静止：", ""])
        for phase in run["phases"]:
            if phase["category"] in {"release", "dropout"} and "stop" in phase["name"]:
                motion = phase["residual_motion"]
                lines.append(
                    f"- {phase['name']}: 稳定停止延迟 {fmt(phase['stop_delay_s'])}s，残余 {fmt(motion['position_max_mm'])}mm / {fmt(motion['orientation_max_deg'])}°。"
                )
            if phase["name"] == "idle_end":
                drift = phase["drift"]
                lines.append(
                    f"- 末尾静止 {drift['duration_s']:.2f}s：漂移 {fmt(drift['position_max_mm'], 6)}mm / {fmt(drift['orientation_max_deg'], 6)}°。"
                )
        lines.append(
            f"- JTC全记录/移动段绝对关节误差P95 {fmt(run['jtc_absolute_error_deg']['p95'], 6)}° / {fmt(run['jtc_moving_error_deg']['p95'], 6)}°；反馈源周期P95 {fmt(run['joint_feedback_period_ms']['p95'])}ms；实际关节速度/配置上限最大比 {fmt(run['joint_speed_limit_ratio'], 5)}。"
        )
        lines.append(
            f"- 原始Servo trajectory发布周期P95 {fmt(run['trajectory_publication_period_ms']['p95'])}ms、末点前瞻P95 {fmt(run['trajectory_point_horizon_ms']['p95'])}ms、收到时相对最近实测关节的参考差P95 {fmt(run['raw_trajectory_reference_gap_deg']['p95'], 6)}°。该参考差含前瞻量；JTC采样desired与actual接近也不能单独证明Servo跟踪准确。"
        )
        clock = run.get("physical_clock")
        if clock is None:
            lines.append("- 未记录物理/clock（GenericSystem不提供物理时钟），real_time_factor=None。")
        elif clock.get("real_time_factor") is None:
            lines.append(f"- 物理时间倍率无法估计：{clock.get('reason')}；不从clock消息条数推断速度。")
        else:
            window = clock["one_second_window_factor"]
            lines.append(
                f"- measurement_start/end内物理时间增量 {clock['physical_duration_s']:.3f}s / host增量 {clock['host_duration_s']:.3f}s，real_time_factor={clock['real_time_factor']:.6f}；去重后{clock['unique_physical_stamps']}条clock，完整1秒窗口{window['n']}个，倍率P05/P50/P95={fmt(window['p05'], 6)}/{fmt(window['p50'], 6)}/{fmt(window['p95'], 6)}。仅作描述，不新增门槛；消息采样数量不等于物理执行步数。"
            )
    lines.extend(
        [
            "",
            "## 重复性",
            "",
            "只在completed=true且协议、GUI、测量/部署哈希一致的组内，按同名保持段比较相对该段起始实际TCP的位移/旋转。目标增量与最终跟踪残差的标准差分开，避免把输入积分闭合差当作执行器重复误差。",
            "",
            "| 组 / 引擎 / 段 | 重复数 | 实际位移标准差 / mm | 目标位移标准差 / mm | 最终跟踪残差标准差 / mm | 实际旋转标准差 / ° |",
            "|---|---:|---|---|---|---|",
        ]
    )
    for item in summary["repeatability"]:
        render = lambda v: "—" if v is None else ", ".join(fmt(x, 5) for x in v)
        lines.append(
            f"| {item['group_id']} / {item['simulator']} / {item['phase']} | {item['n']} | {render(item['position_component_std_mm'])} | {render(item['target_delta_component_std_mm'])} | {render(item['position_following_error_component_std_mm'])} | {render(item['rotation_component_std_deg'])} |"
        )
    lines.extend(
        [
            "",
            "单次局部轨迹、未辨识的执行器/惯量与不完整附件场景不能替代工作空间、接触任务或真机跟随/停止验收。Servo减速码应结合方向、JTC误差和反馈解释；代码1或6本身不是数据故障。",
            "",
        ]
    )
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument(
        "--strict", action="store_true", help="return nonzero for FAIL/INCOMPLETE after writing all reports"
    )
    args = parser.parse_args(argv)
    output = args.output or args.input / "analysis"
    output.mkdir(parents=True, exist_ok=True)
    paths = (
        [args.input]
        if (args.input / "raw.jsonl").is_file()
        else sorted(path.parent for path in args.input.glob("run*/raw.jsonl"))
    )
    if not paths:
        parser.error("no run*/raw.jsonl records found")
    runs = []
    for path in paths:
        run, series = analyze_run(path)
        run["plots"] = [] if args.no_plots else write_plots(run, series, output)
        runs.append(run)
    groups = defaultdict(list)
    for run in runs:
        groups[group_id(run)].append(run)
    group_results = [
        {
            "group_id": identity,
            **group_identity(members[0]),
            "attempt_count": len(members),
            "completed_count": sum(run["completed"] is True for run in members),
            "runs": [run["run_id"] for run in members],
            "status": (
                "FAIL"
                if any(run["status"] == "FAIL" for run in members)
                else ("INCOMPLETE" if any(run["status"] == "INCOMPLETE" for run in members) else "PASS")
            ),
        }
        for identity, members in groups.items()
    ]
    summary = {
        "schema": "rm65-manual-tracking-analysis-v1",
        "status": (
            "FAIL"
            if any(run["status"] == "FAIL" for run in runs)
            else ("INCOMPLETE" if any(run["status"] == "INCOMPLETE" for run in runs) else "PASS")
        ),
        "limits": LIMITS,
        "analysis_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "groups": group_results,
        "runs": runs,
        "repeatability": repeatability(runs),
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8"
    )
    (output / "report.md").write_text(markdown_report(summary), encoding="utf-8")
    print(
        json.dumps(
            {"status": summary["status"], "runs": len(runs), "report": str(output / "report.md")}, ensure_ascii=False
        )
    )
    return 1 if args.strict and summary["status"] != "PASS" else 0


if __name__ == "__main__":
    raise SystemExit(main())
