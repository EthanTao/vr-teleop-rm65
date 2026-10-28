"""Bounded sensor writer and complete-file integrity manifest, separate from arm I/O."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import queue
import threading
import time
import uuid

import numpy as np

from .camera import RGBDFrame
from .peripheral_clock import clock_identity


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


class PeripheralRecorder:
    def __init__(self, root, metadata, *, capacity=8, run_label=""):
        if type(capacity) is not int or not 1 <= capacity <= 256:
            raise ValueError("invalid sensor writer capacity")
        self.path = Path(root) / ("sensors_" + uuid.uuid4().hex)
        self.path.mkdir(parents=True, exist_ok=False)
        (self.path / "frames").mkdir()
        self.metadata = {
            "schema": "rm65_peripherals_v1",
            "session_id": self.path.name,
            "run_label": run_label,
            "clock_identity": clock_identity(),
            "sources": metadata,
            "synchronization": "unsynchronized",
            "hand_eye": None,
            "training_ready": False,
        }
        (self.path / "metadata.json").write_text(json.dumps(self.metadata, allow_nan=False, indent=2), encoding="utf-8")
        self.queue = queue.Queue(maxsize=capacity)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.manifest_lock = threading.Lock()
        self.error = None
        self.dropped = self.written = self.frames = 0
        self.closed = False
        self._write_manifest(False, {})
        self.thread = threading.Thread(target=self._work, name="sensor-writer", daemon=True)
        self.thread.start()

    def _write_manifest(self, complete, files):
        with self.manifest_lock:
            self._replace_manifest(complete and self.error is None, files)

    def _replace_manifest(self, complete, files):
        value = {
            "schema": "rm65_peripherals_manifest_v1",
            "complete": complete,
            "rows": self.written,
            "frames": self.frames,
            "dropped": self.dropped,
            "error": self.error,
            "files": files,
            "training_ready": False,
        }
        temp = self.path / "manifest.json.tmp"
        temp.write_text(json.dumps(value, allow_nan=False, indent=2), encoding="utf-8")
        temp.replace(self.path / "manifest.json")

    def fail(self, reason):
        with self.lock:
            self.error = self.error or str(reason)
        if self.closed:
            self._write_manifest(False, {})

    def append(self, item):
        if isinstance(item, RGBDFrame):
            item.validate()
        else:
            # Copy small event data now; callers cannot mutate queued metadata.
            if not isinstance(item, dict):
                raise ValueError("sensor event must be a dictionary")
            item = json.loads(json.dumps(item, allow_nan=False))
        with self.lock:
            if self.closed or self.error:
                return False
            try:
                self.queue.put_nowait(item)
                return True
            except queue.Full:
                self.dropped += 1
                self.error = "sensor writer queue overflow"
                return False

    def _work(self):
        try:
            with (self.path / "records.jsonl").open("x", encoding="utf-8") as stream:
                while not self.stop.is_set() or not self.queue.empty():
                    try:
                        item = self.queue.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    if isinstance(item, RGBDFrame):
                        path = f"frames/{self.frames:08d}.npz"
                        arrays = {"color_rgb8": item.color, "depth_z16": item.depth}
                        if item.aligned_depth is not None:
                            arrays["aligned_depth_z16"] = item.aligned_depth
                        with (self.path / path).open("xb") as binary:
                            np.savez(binary, **arrays)
                        row = {"kind": "rgbd", "file": path, **item.metadata}
                        self.frames += 1
                    else:
                        row = item
                    stream.write(json.dumps(row, allow_nan=False, separators=(",", ":")) + "\n")
                    self.written += 1
                stream.flush()
        except Exception as exc:
            self.fail(f"sensor write failure: {exc}")

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
        self.stop.set()
        self.thread.join(timeout=5.0)
        if self.thread.is_alive():
            self.fail("sensor writer did not drain within five seconds")
            self._write_manifest(False, {})
            return
        try:
            files = {
                p.relative_to(self.path).as_posix(): {"sha256": file_hash(p), "bytes": p.stat().st_size}
                for p in sorted(self.path.rglob("*"))
                if p.is_file() and p != self.path / "manifest.json"
            }
        except OSError as exc:
            self.fail(f"capture sealing failed: {exc}")
            files = {}
        self._write_manifest(self.error is None and self.dropped == 0 and self.written > 0, files)


def verify_capture(path):
    root = Path(path).resolve()
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema") != "rm65_peripherals_manifest_v1" or manifest.get("complete") is not True:
        raise ValueError("sensor capture is incomplete or unsupported")
    if any(p.is_symlink() for p in root.rglob("*")):
        raise ValueError("symlinks are forbidden in sealed captures")
    actual = {p.relative_to(root).as_posix() for p in root.rglob("*") if p.is_file() and p != root / "manifest.json"}
    if actual != set(manifest["files"]):
        raise ValueError("capture file set differs from manifest")
    for name, expected in manifest["files"].items():
        p = root / name
        if (
            Path(name).is_absolute()
            or ".." in Path(name).parts
            or p.is_symlink()
            or not p.resolve().is_relative_to(root)
        ):
            raise ValueError("unsafe capture file path")
        if p.stat().st_size != expected["bytes"] or file_hash(p) != expected["sha256"]:
            raise ValueError(f"capture hash mismatch: {name}")
    rows = [json.loads(line) for line in (root / "records.jsonl").read_text(encoding="utf-8").splitlines()]
    frames = [row for row in rows if row.get("kind") == "rgbd"]
    if len(rows) != manifest["rows"] or len(frames) != manifest["frames"]:
        raise ValueError("capture row/frame count mismatch")
    if {row["file"] for row in frames} != {n for n in actual if n.startswith("frames/")}:
        raise ValueError("orphaned or missing frame file")
    for row in frames:
        with np.load(root / row["file"], allow_pickle=False) as arrays:
            RGBDFrame(
                arrays["color_rgb8"],
                arrays["depth_z16"],
                row,
                arrays["aligned_depth_z16"] if "aligned_depth_z16" in arrays else None,
            ).validate()
    return manifest
