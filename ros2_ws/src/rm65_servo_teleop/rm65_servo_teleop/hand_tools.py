"""Runnable offline rehearsal and explicit read-only preflight, without ROS."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import yaml

from .hand_protocol import HandError, probe
from .hand_session import HandProfile, HandSession, MockHand


def demo(profile: HandProfile) -> list[dict]:
    hand = MockHand(profile)
    session = HandSession(profile)
    rows = []
    now = 0.0

    def tick():
        nonlocal now
        now = round(now + 0.05, 6)
        session.observe(hand.poll(now), now)
        session.flush_mock(hand, now)
        rows.extend(session.events)
        session.events.clear()

    def action(name):
        envelope = {
            "action": name,
            "session_id": session.session_id,
            "sequence": session.last_command_sequence + 1,
            "expected_revision": session.revision,
        }
        if not session.command(envelope, now):
            raise HandError(f"demo action rejected: {name}")
        tick()

    def wait_for(stage):
        for _ in range(200):
            if session.stage == stage:
                return
            if session.stage == "fault":
                raise HandError(session.fault_reason)
            tick()
        raise HandError(f"demo failed to reach {stage}")

    tick()  # Initial feedback, zero writes.
    action("prepare")
    wait_for("ready")
    action("grasp")
    wait_for("awaiting_grasp_confirmation")
    action("confirm_grasp")  # Synthetic operator action, never contact-force inference.
    action("begin_insert")
    for _ in range(10):
        tick()  # No new targets during hold/insertion and no arm command.
    action("finish_insert")
    action("release")
    wait_for("released")
    rows.append(
        {
            "kind": "summary",
            **session.status(now),
            "mock_position_writes": hand.write_count,
            "automatic_arm_motion": False,
            "insertion_physics_simulated": False,
        }
    )
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation", required=True)
    rehearsal = sub.add_parser("demo", help="offline six-channel task rehearsal; never connects to a robot")
    rehearsal.add_argument("--profile", type=Path, required=True)
    rehearsal.add_argument("--output", type=Path)
    readonly = sub.add_parser("probe", help="explicit standalone read-only RM65 end-port preflight")
    readonly.add_argument("--arm-ip", required=True)
    readonly.add_argument("--device", type=int, required=True)
    readonly.add_argument("--protocol-major", type=int, choices=[1, 2], required=True)
    readonly.add_argument("--timeout", type=float, default=0.5)
    readonly.add_argument("--execute-readonly", action="store_true", help="connect and read; default only previews")
    readonly.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        # Reserve the output before any optional I/O; never overwrite existing evidence.
        with args.output.open("x", encoding="utf-8") if args.output else _Stdout() as output:
            if args.operation == "demo":
                profile = HandProfile.from_dict(yaml.safe_load(args.profile.read_text(encoding="utf-8")))
                for row in demo(profile):
                    output.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
            else:
                from .hand_protocol import integer, read_request

                integer(args.device, 1, 247, "device")
                if args.execute_readonly:
                    result = probe(args.arm_ip, args.device, args.protocol_major, timeout_s=args.timeout)
                else:
                    result = {
                        "preview_only": True,
                        "network_opened": False,
                        "hardware_writes": False,
                        "arm_ip": args.arm_ip,
                        "identity_request": read_request("identity", args.device),
                    }
                output.write(json.dumps(result, ensure_ascii=False, allow_nan=False) + "\n")
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 2
    return 0


class _Stdout:
    def __enter__(self):
        return sys.stdout

    def __exit__(self, *_):
        pass


if __name__ == "__main__":
    raise SystemExit(main())
