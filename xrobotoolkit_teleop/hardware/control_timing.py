"""Absolute-deadline scheduling and bounded loop timing statistics."""

from collections import deque
import math

import numpy as np


class FixedRateScheduler:
    """Advance fixed deadlines without issuing catch-up bursts."""

    def __init__(self, rate_hz: float, start_time_s: float) -> None:
        self.rate_hz = self._positive_finite(rate_hz, "rate_hz")
        self.period_s = 1.0 / self.rate_hz
        self.next_deadline_s = self._finite(start_time_s, "start_time_s")
        self.missed_periods = 0

    @staticmethod
    def _finite(value: float, name: str) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be finite") from exc
        if not math.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result

    @classmethod
    def _positive_finite(cls, value: float, name: str) -> float:
        result = cls._finite(value, name)
        if result <= 0.0:
            raise ValueError(f"{name} must be positive")
        return result

    def advance(self, now_s: float) -> float:
        now = self._finite(now_s, "now_s")
        self.next_deadline_s += self.period_s
        while self.next_deadline_s <= now:
            self.next_deadline_s += self.period_s
            self.missed_periods += 1
        return self.next_deadline_s


class LoopTimingStats:
    """Keep a bounded sample window for control-loop diagnostics."""

    def __init__(self, max_samples: int = 4096) -> None:
        if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples <= 0:
            raise ValueError("max_samples must be a positive integer")
        self._period_s: deque[float] = deque(maxlen=max_samples)
        self._send_s: deque[float] = deque(maxlen=max_samples)
        self._udp_age_s: deque[float] = deque(maxlen=max_samples)
        self.deadline_misses = 0

    def record(self, period_s: float, send_s: float, udp_age_s: float) -> None:
        values = np.asarray([period_s, send_s, udp_age_s], dtype=float)
        if not np.all(np.isfinite(values)) or np.any(values < 0.0):
            raise ValueError("timing samples must be finite and non-negative")
        self._period_s.append(float(period_s))
        self._send_s.append(float(send_s))
        self._udp_age_s.append(float(udp_age_s))

    def summary(self) -> dict[str, float | int]:
        periods_ms = np.asarray(self._period_s, dtype=float) * 1000.0
        sends_ms = np.asarray(self._send_s, dtype=float) * 1000.0
        udp_ms = np.asarray(self._udp_age_s, dtype=float) * 1000.0

        def percentile(values: np.ndarray, q: float) -> float:
            return float(np.percentile(values, q)) if values.size else 0.0

        return {
            "samples": len(self._period_s),
            "period_p50_ms": percentile(periods_ms, 50),
            "period_p95_ms": percentile(periods_ms, 95),
            "period_p99_ms": percentile(periods_ms, 99),
            "period_max_ms": float(np.max(periods_ms)) if periods_ms.size else 0.0,
            "send_max_ms": float(np.max(sends_ms)) if sends_ms.size else 0.0,
            "udp_age_max_ms": float(np.max(udp_ms)) if udp_ms.size else 0.0,
            "deadline_misses": int(self.deadline_misses),
        }
