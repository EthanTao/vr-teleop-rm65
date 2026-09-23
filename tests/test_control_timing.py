import pytest

from xrobotoolkit_teleop.hardware.control_timing import FixedRateScheduler, LoopTimingStats


def test_fixed_rate_scheduler_skips_missed_deadlines_without_catch_up_burst():
    scheduler = FixedRateScheduler(rate_hz=50.0, start_time_s=10.0)

    deadline = scheduler.advance(10.071)

    assert deadline == pytest.approx(10.08)
    assert scheduler.missed_periods == 3


def test_timing_stats_report_bounded_millisecond_values():
    stats = LoopTimingStats(max_samples=2)
    stats.record(0.02, 0.003, 0.005)
    stats.record(0.03, 0.004, 0.01)
    stats.record(0.04, 0.005, 0.02)

    summary = stats.summary()

    assert summary["samples"] == 2
    assert summary["period_p50_ms"] == pytest.approx(35.0)
    assert summary["send_max_ms"] == pytest.approx(5.0)
    assert summary["udp_age_max_ms"] == pytest.approx(20.0)
