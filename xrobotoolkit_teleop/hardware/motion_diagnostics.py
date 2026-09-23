"""Opt-in, bounded asynchronous motion tracing. No disk I/O in record()."""

from collections import deque
import json
import math
from pathlib import Path
import queue
import threading

import numpy as np


def _json_safe(value):
    """Retain fault rows with non-finite sensor values as JSON null."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


def interval_summary(values):
    values = np.asarray(values, dtype=float) * 1000.0
    return {
        "count": int(values.size),
        **{f"p{q}_ms": float(np.percentile(values, q)) if values.size else None for q in (50, 95, 99)},
        "max_ms": float(np.max(values)) if values.size else None,
        "over_30_ms": int(np.count_nonzero(values > 30.0)),
    }


class MotionStreamStats:
    """Windowed observed rates, not network arrival rates or controller execution."""

    def __init__(self):
        self.send_intervals = deque(maxlen=4096)
        self.transport_intervals = deque(maxlen=4096)
        self.xr_intervals = deque(maxlen=4096)
        self.send_durations = deque(maxlen=4096)
        self.feedback_ages = deque(maxlen=4096)
        self.xr_seconds = {}
        self.xr_changes = deque()
        self.last_send = None
        self.last_transport = None
        self.last_xr_time = None
        self.last_xr_stamp = None
        self.previous_cycle = None

    def observe(self, row):
        now = row["t_monotonic"]
        contiguous = self.previous_cycle is not None and row["cycle_index"] == self.previous_cycle + 1
        self.previous_cycle = row["cycle_index"]
        duration = row.get("send_duration_s")
        age = row.get("feedback_age_s")
        if duration is not None and math.isfinite(duration):
            self.send_durations.append(duration)
        if age is not None and math.isfinite(age):
            self.feedback_ages.append(age)
        second = math.floor(now)
        self.xr_seconds.setdefault(second, 0)
        for old_second in list(self.xr_seconds):
            if old_second < second - 5:
                del self.xr_seconds[old_second]
        send = row.get("send_started_monotonic")
        transport = row.get("transport") or {}
        wire = transport.get("started_monotonic") if transport.get("sendall_completed") else None
        for value, previous, intervals in (
            (send, self.last_send, self.send_intervals),
            (wire, self.last_transport, self.transport_intervals),
        ):
            if contiguous and value is not None and previous is not None:
                intervals.append(value - previous)
        # Idle, failed or skipped sends break segments; intentional pauses are not holes.
        self.last_send, self.last_transport = send, wire
        stamp = row.get("xr_timestamp_ns")
        if stamp is not None and stamp > 0 and stamp != self.last_xr_stamp:
            if self.last_xr_time is not None:
                self.xr_intervals.append(now - self.last_xr_time)
            self.last_xr_time, self.last_xr_stamp = now, stamp
            self.xr_changes.append(now)
            self.xr_seconds[second] += 1
        while self.xr_changes and self.xr_changes[0] <= now - 1.0:
            self.xr_changes.popleft()

    def summary(self):
        result = {
            "send_attempt_interval": interval_summary(self.send_intervals),
            "sendall_interval": interval_summary(self.transport_intervals),
            "xr_observed_interval": interval_summary(self.xr_intervals),
            "xr_changes_last_second": len(self.xr_changes),
            "xr_changes_per_monotonic_second": dict(self.xr_seconds),
            "send_duration": interval_summary(self.send_durations),
            "feedback_age": interval_summary(self.feedback_ages),
        }
        self.send_intervals.clear()
        self.transport_intervals.clear()
        self.xr_intervals.clear()
        self.send_durations.clear()
        self.feedback_ages.clear()
        return result


class MotionLog:
    """500-row batches; bounded queue; overflow/error explicitly invalidates trace."""

    def __init__(self, path: str, *, queue_size=10000):
        if queue_size <= 0:
            raise ValueError("queue_size must be positive")
        # Exclusive creation preserves earlier evidence and validates path before motion.
        self._file = Path(path).open("x", encoding="utf-8", buffering=1024 * 1024)
        self._queue = queue.Queue(maxsize=queue_size)
        self.dropped_rows = 0
        self.error = None
        self._closed = False
        self._thread = threading.Thread(target=self._worker, name="rm65-motion-log", daemon=True)
        self._thread.start()

    def _enqueue(self, item):
        if self._closed:
            raise RuntimeError("motion log is closed")
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.dropped_rows += 1

    def record(self, row):
        self._enqueue(("cycle", row))

    def summary(self, counters):
        self._enqueue(("summary", counters))

    def _worker(self):
        stats = MotionStreamStats()
        batch = []

        def flush():
            if not batch:
                return
            try:
                self._file.write("".join(json.dumps(_json_safe(row), allow_nan=False) + "\n" for row in batch))
                self._file.flush()
            except Exception as exc:
                self.error = str(exc)
            batch.clear()

        while True:
            item = self._queue.get()
            if item is None:
                break
            kind, row = item
            if kind == "cycle":
                stats.observe(row)
                batch.append(row)
                if len(batch) >= 500:
                    flush()
            else:
                summary = {
                    "type": "summary",
                    **row,
                    **stats.summary(),
                    "log_dropped_rows": self.dropped_rows,
                    "log_error": self.error,
                }
                batch.append(summary)
                # Only this worker prints new periodic diagnostics, never the control loop.
                try:
                    print("[MOTION TIMING] " + json.dumps(summary, allow_nan=False))
                except Exception as exc:
                    self.error = str(exc)
        batch.append({"type": "log_end", "dropped_rows": self.dropped_rows, "error": self.error})
        flush()
        self._file.close()

    def close(self):
        if not self._closed:
            self._closed = True
            self._queue.put(None, timeout=5.0)
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                raise RuntimeError("motion log did not drain within 5 seconds; trace may be incomplete")
            if self.error or self.dropped_rows:
                raise RuntimeError(f"incomplete motion log: error={self.error}, dropped={self.dropped_rows}")
