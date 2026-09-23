"""Print the recommended singularity-free TCP picks per radius band from a scan JSON."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", type=Path, default=Path("docs/_singularity_free_tcp_scan.json"))
    parser.add_argument("--bands", type=float, nargs="*", default=[0.35, 0.45, 0.55, 0.65, 0.78])
    args = parser.parse_args()
    data = json.loads(args.json.read_text(encoding="utf-8"))
    sweep = {round(entry["band_m"][0], 3): entry for entry in data["radial_sweep"]}
    print(f"{'radius band':>12} | {'q_deg (J1..J6)':<46} | tcp xyz [m]                | ratio | margin | self [m]")
    for low, high in zip(args.bands[:-1], args.bands[1:]):
        key = round(low, 3)
        entry = sweep.get(key)
        if entry is None:
            continue
        q = ", ".join(f"{value:7.2f}" for value in entry["joint_deg"])
        xyz = ", ".join(f"{value:+.4f}" for value in entry["tcp_xyz_m"])
        print(
            f"{low:.2f}-{high:.2f} m | [{q}] | [{xyz}] | {entry['ratio']:.4f} |"
            f" {entry['worst_margin']:.2f}x | {entry['self_clearance_m']:.3f}"
        )
        print(f"{'':>12} |   quat wxyz {entry['tcp_quat_wxyz']} | rpy_deg {entry['tcp_rpy_deg']}")
        print(
            f"{'':>12} |   J3={entry['j3_deg']:+.2f} deg ({entry['j3_over_slowdown']:.2f}x slowdown),"
            f" J5={entry['j5_deg']:+.2f} deg ({entry['j5_over_slowdown']:.2f}x),"
            f" ratio {entry['ratio_over_slowdown']:.2f}x slowdown / {entry['ratio_over_danger']:.1f}x danger"
        )
    joint_limits = np.deg2rad([[-178, 178], [-130, 130], [-135, 135], [-178, 178], [-128, 128], [-360, 360]])
    for low, high in zip(args.bands[:-1], args.bands[1:]):
        entry = sweep.get(round(low, 3))
        if entry is None:
            continue
        q = np.deg2rad(entry["joint_deg"])
        headroom = np.minimum(q - joint_limits[:, 0], joint_limits[:, 1] - q)
        print(
            f"  band {low:.2f}-{high:.2f} m joint-limit headroom [deg]:"
            f" {np.round(np.rad2deg(headroom), 1).tolist()}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
