"""Camera acquisition worker, bounded disk queue, latched errors; no arm I/O."""

import threading
import time

from .camera import CameraConfig, MockCamera, RealSenseCamera
from .peripheral_recording import PeripheralRecorder


class CameraRuntime:
    def __init__(self, source, config, root, *, run_label="", camera=None):
        if source not in {"mock", "realsense"}:
            raise ValueError("camera source must be mock or realsense")
        self.source, self.config, self.root, self.run_label = source, config, root, run_label
        self.camera = camera or (MockCamera(config) if source == "mock" else RealSenseCamera(config))
        self.recorder = None
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.error = None
        self.frames = 0
        self.status = {"state": "starting", "source": source, "synthetic": source == "mock"}
        self.thread = None

    def start(self):
        if self.thread is not None:
            raise ValueError("camera runtime cannot be restarted")
        self.thread = threading.Thread(target=self._work, name="d435i-acquisition", daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return {
                **self.status,
                "error": self.error,
                "frames": self.frames,
                "capture_path": str(self.recorder.path) if self.recorder else None,
                "run_label": self.run_label,
                "training_ready": False,
                "hand_eye": None,
            }

    def _work(self):
        previous = None
        try:
            self.camera.start()
            self.recorder = PeripheralRecorder(
                self.root, {"camera": self.camera.metadata}, capacity=self.config.queue_size, run_label=self.run_label
            )
            while not self.stop.is_set():
                started = time.monotonic()
                frame = self.camera.read().validate()
                m = frame.metadata
                if m["synthetic"] != (self.source == "mock"):
                    raise ValueError("camera source label mismatch")
                gaps = {"color": 0, "depth": 0}
                if previous:
                    if m["timestamp_domain"] != previous["timestamp_domain"]:
                        raise ValueError("camera clock domain changed")
                    for name in ("color", "depth"):
                        if m[f"{name}_timestamp_ms"] <= previous[f"{name}_timestamp_ms"]:
                            raise ValueError("camera timestamp repeated or regressed")
                        step = m[f"{name}_frame_number"] - previous[f"{name}_frame_number"]
                        if step <= 0:
                            raise ValueError("camera frame replay/regression")
                        gaps[name] = step - 1
                    if m["host_received_monotonic_s"] <= previous["host_received_monotonic_s"]:
                        raise ValueError("camera host receive time regressed")
                m["source_frame_gaps"] = gaps
                if not self.recorder.append(frame):
                    raise RuntimeError(self.recorder.error or "camera recorder closed")
                previous = dict(m)
                with self.lock:
                    self.frames += 1
                    self.status = {
                        "state": "recording",
                        "source": self.source,
                        "synthetic": self.source == "mock",
                        "last_frame": m,
                        "device": self.camera.metadata,
                    }
                self.stop.wait(max(0, 1 / self.config.fps - (time.monotonic() - started)))
        except Exception as exc:
            self.error = f"camera capture failed: {exc}"
            if self.recorder:
                self.recorder.fail(self.error)
        finally:
            try:
                self.camera.close()
            except Exception as exc:
                self.error = self.error or f"camera close failed: {exc}"
                if self.recorder:
                    self.recorder.fail(self.error)
            if self.recorder:
                self.recorder.close()
                self.error = self.error or self.recorder.error
            with self.lock:
                self.status = {**self.status, "state": "fault" if self.error else "closed"}

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=self.config.timeout_ms / 1000 + 7)
            if self.thread.is_alive():
                self.error = "camera shutdown deadline exceeded"
                if self.recorder:
                    self.recorder.fail(self.error)
