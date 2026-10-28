"""Offline same-host arrival pairing. NEVER interprets a target as measurement."""

from bisect import bisect_left
import json
import math
from pathlib import Path

from .peripheral_recording import verify_capture


def pair_capture(capture, journal, output, *, max_delta_s=0.05):
    if type(max_delta_s) not in (int, float) or not math.isfinite(max_delta_s) or not 0 < max_delta_s <= 1:
        raise ValueError("pairing bound must be in (0,1] seconds")
    capture, journal, output = Path(capture), Path(journal), Path(output)
    verify_capture(capture)
    camera_meta = json.loads((capture / "metadata.json").read_text(encoding="utf-8"))
    arm_meta = json.loads((journal / "session.json").read_text(encoding="utf-8"))
    identity = camera_meta.get("clock_identity")
    if not identity or identity != arm_meta.get("clock_identity"):
        raise ValueError("clock identities differ/unknown; a measured cross-host clock bridge is required")
    if arm_meta.get("complete") is not True or arm_meta.get("mode") not in {"hardware", "hardware-dry-run"}:
        raise ValueError("complete hardware arm journal required; sim/targets cannot stand in for measurement")
    if camera_meta["sources"].get("camera", {}).get("synthetic") is not False:
        raise ValueError("synthetic camera cannot be paired as real observation")
    samples, seen = [], set()
    with (journal / "records.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("topic") != "/rm65/driver/feedback_snapshot" or row.get("quality", {}).get("valid") is not True:
                continue
            value = row["value"]
            packet = json.loads(value["data"])
            key = (packet.get("clock_id"), packet.get("session"), packet.get("packet_id"))
            if None in key or key in seen or packet.get("parse_error") != 0:
                continue
            if len(packet.get("position_rad", [])) != 6 or len(packet.get("velocity_rad_s", [])) != 6:
                continue
            if not all(
                type(v) in (int, float) and math.isfinite(v) for v in packet["position_rad"] + packet["velocity_rad_s"]
            ):
                continue
            seen.add(key)
            stamp = row["received_monotonic_s"]
            if type(stamp) not in (int, float) or not math.isfinite(stamp):
                raise ValueError("invalid journal receive time")
            samples.append((stamp, packet))
    samples.sort(key=lambda x: x[0])
    if not samples:
        raise ValueError("no valid measured UDP feedback packets")
    times = [s[0] for s in samples]
    # Reserve output only after validating inputs; never overwrite existing evidence.
    with (
        output.open("x", encoding="utf-8") as destination,
        (capture / "records.jsonl").open(encoding="utf-8") as source,
    ):
        for line in source:
            frame = json.loads(line)
            if frame.get("kind") != "rgbd":
                continue
            if frame.get("synthetic") is not False:
                raise ValueError("synthetic frame cannot be paired as physical observation")
            stamp = frame["host_received_monotonic_s"]
            index = bisect_left(times, stamp)
            candidates = [k for k in (index - 1, index) if 0 <= k < len(times)]
            closest = min(candidates, key=lambda k: abs(times[k] - stamp))
            delta = times[closest] - stamp
            accepted = abs(delta) <= max_delta_s
            destination.write(
                json.dumps(
                    {
                        "schema": "rm65_arrival_pair_v1",
                        "frame_file": frame["file"],
                        "capture_session": camera_meta["session_id"],
                        "clock_identity": identity,
                        "pairing": "host_and_ros_arrival_only",
                        "exposure_synchronized": False,
                        "arrival_delta_s": delta,
                        "matched": accepted,
                        "max_delta_s": max_delta_s,
                        "feedback": samples[closest][1] if accepted else None,
                        "calibrated_pose": None,
                        "training_ready": False,
                    },
                    allow_nan=False,
                )
                + "\n"
            )
    return output
