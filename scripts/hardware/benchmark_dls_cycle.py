#!/usr/bin/env python3
"""Break down the per-cycle cost of the default DLS path. Read-only, no motion.

Run inside the runtime container (the same interpreter ``teleop.py`` uses), e.g.:

    sudo docker exec -it xrobo-vr-teleop bash -c \
      'cd /workspace && python scripts/hardware/benchmark_dls_cycle.py --arm-host 192.168.10.18'

It performs the same read-only startup handshake as
``scripts/hardware/check_realman_dls_frames.py`` (``get_arm_software_info``,
``get_DH_data``, ``get_current_tool_frame``, UDP feedback), then measures, each
with the UDP feedback thread running and paused:

* raw ``rm_algo_forward_kinematics`` (inside the SDK),
* ``ControllerFrameKinematics.forward`` with the real work/tool frames,
* the same adapter with identity frames (no pose composition),
* the same adapter fed by an in-process synthetic RM65 chain (no SDK at all),
* complete ``DampedIK.step`` calls, including the FK-pose count per step.

Every adapter row is also split into "time inside the official SDK call" and
"time in the adapter's own Python", by wrapping the SDK object and accumulating
its wall time. That split is the point of the benchmark: "the adapter is slow"
and "the SDK call inside the adapter is slow" need opposite fixes.

Options:

``--stop-udp-during-step``  repeat the whole set with the feedback thread paused,
                            to separate GIL/scheduling contention from compute.
``--gc-off-adapter``        repeat the adapter rows with the cyclic collector
                            disabled, to test collector pressure.
``--self-test``             exercise the measurement and reporting path with
                            fakes; needs neither the arm nor the SDK.

Nothing is ever sent and no controller configuration is written.
"""

from __future__ import annotations

import argparse
import gc
import os
from pathlib import Path
import statistics
import sys
import time
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from xrobotoolkit_teleop.hardware.damped_ik import ControllerFrameKinematics, DampedIK, DLSBudgetExceeded
from xrobotoolkit_teleop.hardware.interface.realman_rm65 import RealmanRM65Interface
from xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller import RealmanRM65SafeTeleopController
from xrobotoolkit_teleop.hardware.realman_udp_feedback import ArmFeedback
from xrobotoolkit_teleop.hardware.rm65_model import RM65_B_SPEC
from xrobotoolkit_teleop.hardware.singularity_avoidance import RM65_CHAIN, rm65_forward_kinematics


class ReadOnlyArm(RealmanRM65Interface):
    """Refuse anything that is not a read-only query."""

    def _sendall_observed(self, sock, data, payload):
        allowed = {"get_current_arm_state", "get_arm_software_info", "get_DH_data", "get_current_tool_frame"}
        if payload.get("command") not in allowed:
            raise RuntimeError(f"read-only benchmark rejected command: {payload}")
        return super()._sendall_observed(sock, data, payload)


class SyntheticOfficialFK:
    """Same contract as RealmanOfficialRemoteIK.forward, computed locally."""

    def forward(self, joint_rad):
        matrix = rm65_forward_kinematics(np.asarray(joint_rad, dtype=float), RM65_CHAIN)
        trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
        w = np.sqrt(max(0.0, 1.0 + trace)) / 2.0
        quat = np.array(
            [
                w,
                (matrix[2, 1] - matrix[1, 2]) / (4.0 * w),
                (matrix[0, 2] - matrix[2, 0]) / (4.0 * w),
                (matrix[1, 0] - matrix[0, 1]) / (4.0 * w),
            ]
        )
        return matrix[:3, 3].copy(), quat / np.linalg.norm(quat)


class SdkTimeProbe:
    """Wrap the official kinematics and attribute wall time to the SDK call itself.

    Without this split, "the adapter costs 1 ms" and "the SDK call inside the
    adapter costs 1 ms" look identical. The controller's UDP thread keeps running
    here on purpose: pausing it during the loop would itself change the GIL
    behaviour and hide the effect that produced the earlier 6.14x ratio.
    """

    def __init__(self, inner):
        self.inner = inner
        self.elapsed = 0.0
        self.calls = 0

    def forward(self, joint_rad):
        start = time.perf_counter()
        result = self.inner.forward(joint_rad)
        self.elapsed += time.perf_counter() - start
        self.calls += 1
        return result

    def reset(self) -> None:
        self.elapsed = 0.0
        self.calls = 0


def _summary(label: str, samples: list[float]) -> float:
    ordered = sorted(samples)
    p95 = ordered[min(len(ordered) - 1, int(round(0.95 * (len(ordered) - 1))))]
    median = statistics.median(samples)
    print(
        f"  {label:46s} n={len(samples):4d} "
        f"median={median * 1000:8.3f}ms "
        f"p95={p95 * 1000:8.3f}ms "
        f"max={max(samples) * 1000:8.3f}ms"
    )
    return median


def _time_calls(repeat: int, call) -> list[float]:
    call()  # warm-up, excluded from the statistics
    samples = []
    for _ in range(repeat):
        start = time.perf_counter()
        call()
        samples.append(time.perf_counter() - start)
    return samples


def _measure(
    controller,
    seed: np.ndarray,
    last_feedback,
    *,
    repeat_fk: int,
    repeat_cycle: int,
    target_step_mm: float,
    pause_udp: bool,
    gc_off_adapter: bool,
    medians: dict,
    shared: dict,
) -> tuple[float, float, int]:
    """Run every measurement block and fill ``medians``/``shared``. Never sends."""
    receiver = controller.feedback_receiver
    dls = controller.dls
    frames = dls.kinematics
    probe = frames.official
    if not isinstance(probe, SdkTimeProbe):
        raise RuntimeError("the SDK time probe is not installed on the DLS kinematics")
    identity_frames = ControllerFrameKinematics(probe, None, None)
    synthetic_frames = ControllerFrameKinematics(SyntheticOfficialFK(), None, None)

    def timed_composed():
        dls.begin_cycle()
        frames.forward(seed)

    def timed_identity():
        dls.begin_cycle()
        identity_frames.forward(seed)

    def timed_synthetic():
        dls.begin_cycle()
        synthetic_frames.forward(seed)

    def collect_fk(tag: str, gc_off: bool = False) -> None:
        """Time every FK flavour once under the current thread/collector state."""
        for label, call in (
            ("official FK (raw, SDK)", lambda: probe.inner.forward(seed)),
            ("adapter forward (real frames)", timed_composed),
            ("adapter forward (identity frames)", timed_identity),
            ("adapter forward (synthetic chain)", timed_synthetic),
        ):
            row = f"{label} [gc off]" if gc_off else label
            if gc_off:
                probe.reset()
            medians[(tag, row)] = _summary(f"  [{tag}] {row}", _time_calls(repeat_fk, call))
            if label == "adapter forward (real frames)":
                # One SDK forward per timed call, plus the pre-warm miss.
                shared[(tag, row)] = (probe.elapsed / max(probe.calls, 1), probe.calls)

    # A: before the controller connection, with the SDK's default DH. The real DH
    # is only known after startup, so this block is a baseline only.
    print("\n[before startup: default SDK DH, thread not started]")
    for _ in range(50):  # warm the first-call costs out of the measurement
        probe.inner.forward(seed)
    collect_fk("pre-startup")

    print(f"\nmeasured joints_deg = {np.round(np.rad2deg(seed), 3).tolist()}")
    print(f"udp valid frames so far = {getattr(receiver, 'valid_packet_count', 'n/a')}")
    print("\n[after startup: controller DH + finger tool, thread running]")
    collect_fk("running")

    if pause_udp:
        receiver.stop()
        print("\n[after startup: same call set, UDP thread paused]")
        collect_fk("paused")
        if gc_off_adapter:
            print("\n[after startup: adapter rows with the cyclic collector disabled]")
            gc.disable()
            try:
                collect_fk("paused-nogc", gc_off=True)
            finally:
                gc.enable()
        receiver.start()
        receiver._store_latest(last_feedback)

    # C: complete guarded DLS steps
    target = frames.forward(seed)
    command = target[0] + np.array([target_step_mm / 1000.0, 0.0, 0.0])
    speed_limits = RM65_B_SPEC.max_joint_velocity_rad_s * controller.max_joint_speed_ratio

    def one_step() -> tuple[float, int, str]:
        dls.begin_cycle()
        start = time.perf_counter()
        try:
            result = dls.step(
                command,
                target[1],
                seed,
                controller.dt,
                speed_limits,
                previous_velocity=controller._dls_velocity,
                max_linear_speed=controller.max_linear_velocity_m_s,
                max_angular_speed=controller.max_angular_velocity_rad_s,
            )
            status = result.status
        except DLSBudgetExceeded as exc:
            status = f"hold_budget:{exc.reason}"
        return time.perf_counter() - start, dls.fk_calls, status

    def collect_step(pause: bool) -> tuple[list[float], list[int], list[str], list[float]]:
        times, calls, statuses, sdk_times = [], [], [], []
        for _ in range(repeat_cycle):
            if pause:
                receiver.stop()
            try:
                probe.reset()
                elapsed, count, status = one_step()
                sdk_times.append(probe.elapsed)
            finally:
                if pause:
                    receiver.start()
                    receiver._store_latest(last_feedback)
            times.append(elapsed)
            calls.append(count)
            statuses.append(status)
        return times, calls, statuses, sdk_times

    print(
        f"\n[DampedIK.step, target step {target_step_mm:.1f} mm in controller x; "
        f"budget={dls.compute_budget_s * 1000:.1f}ms/{dls.max_fk_calls} FK/"
        f"{dls.max_candidate_attempts} candidates]"
    )
    times, calls, statuses, sdk_times = collect_step(False)
    running_step = _summary("  [running] step (Jacobian + bounded candidate)", times)
    fk_per_step = max(calls)
    print(
        f"    distinct FK poses per step: min={min(calls)} max={max(calls)}  statuses={sorted(set(statuses))}\n"
        f"    SDK forward time inside the step: median={statistics.median(sdk_times) * 1000:.3f}ms "
        f"({statistics.median(sdk_times) / running_step * 100:.1f}% of the step)"
    )
    paused_step = running_step
    if pause_udp:
        times, calls, statuses, sdk_times = collect_step(True)
        paused_step = _summary("  [paused]  step (Jacobian + bounded candidate)", times)
        print(
            f"    distinct FK poses per step: min={min(calls)} max={max(calls)}  statuses={sorted(set(statuses))}\n"
            f"    SDK forward time inside the step: median={statistics.median(sdk_times) * 1000:.3f}ms "
            f"({statistics.median(sdk_times) / paused_step * 100:.1f}% of the step)"
        )
        print(f"    paused/running ratio: {paused_step / max(running_step, 1e-9):.2f}x")

    _report(medians, shared, fk_per_step, controller.dt, running_step, paused_step)
    return running_step, paused_step, fk_per_step


def _report(medians: dict, shared: dict, fk_per_step: int, dt: float, running_step: float, paused_step: float) -> None:
    print("\n[attribution: where each per-call millisecond goes]")
    header = f"  {'state':12s} {'row':28s} {'total':>10s} {'sdk/call':>10s} {'adapter':>10s} {'sdk share':>10s}"
    print(header)
    for tag in ("pre-startup", "running", "paused", "paused-nogc"):
        raw = medians.get((tag, "official FK (raw, SDK)"))
        if raw is not None:
            print(
                f"  {tag:12s} {'official FK (raw, SDK)':28s} {raw * 1000:9.3f}ms "
                f"{raw * 1000:9.3f}ms {0.0:9.3f}ms {100.0:9.0f}%"
            )
        for row, label in (
            ("adapter forward (real frames)", "real frames"),
            ("adapter forward (real frames) [gc off]", "real frames [gc off]"),
            ("adapter forward (identity frames)", "identity frames"),
            ("adapter forward (synthetic chain)", "synthetic (no SDK)"),
        ):
            total = medians.get((tag, row))
            if total is None:
                continue
            sdk = shared.get((tag, row))
            if sdk is None:
                sdk_note, added, share_note = "n/a", "n/a", "n/a"
            else:
                sdk_note = f"{sdk[0] * 1000:.3f}ms"
                added = f"{(total - sdk[0]) * 1000:.3f}ms"
                share_note = f"{sdk[0] / total * 100:.0f}%"
            print(f"  {tag:12s} {label:28s} {total * 1000:9.3f}ms {sdk_note:>10s} {added:>10s} {share_note:>10s}")
    print(
        f"\n  step totals: running={running_step * 1000:.1f}ms paused={paused_step * 1000:.1f}ms "
        f"budget={dt * 1000:.1f}ms"
    )
    pose = shared.get(("paused", "adapter forward (real frames)"))
    if pose is None:
        pose = shared.get(("running", "adapter forward (real frames)"))
    if fk_per_step and pose is not None and pose[0] > 0:
        print(
            f"  extrapolation: {fk_per_step} FK poses/step x {pose[0] * 1000:.3f}ms per pose = "
            f"{fk_per_step * pose[0] * 1000:.1f}ms per {dt * 1000:.1f}ms budget"
        )


def _add_pair(left: float, right: float) -> float:
    """Trivial callable used to price a plain Python function call."""
    return left + right


def _cpu_sample() -> tuple[float, float, float]:
    """Return (process CPU seconds, wall seconds, clock ticks/since boot)."""
    times = os.times()
    cpu = times.user + times.system
    ticks = None
    try:
        with open("/proc/uptime", encoding="ascii") as handle:
            ticks = float(handle.read().split()[0])
    except OSError:
        ticks = None
    return cpu, time.perf_counter(), ticks if ticks is not None else float("nan")


def _micro(repeat: int) -> int:
    """Time the primitive numpy operations the adapter and kinematics rely on.

    The container's per-operation cost is the leading explanation for the
    per-call overhead, so measure it directly instead of inferring it. The first
    two rows are pure Python with no NumPy involved: if those are also an order
    of magnitude slower than the dev-PC reference, the whole interpreter is slow
    on this board, not NumPy. CPU seconds versus wall seconds per row says
    whether the process is being throttled or descheduled. ``/proc/uptime``
    shows whether the board is suspending between calls.
    """
    import os
    import platform

    print("\n[environment]")
    print(f"  python {platform.python_version()} on {platform.platform()}")
    print(f"  numpy {np.__version__} from {np.__file__}")
    for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        print(f"  {variable}={os.environ.get(variable, '<unset>')}")

    small = np.arange(6, dtype=float)
    matrix = np.eye(4)

    def python_call_chain():
        value = 0.0
        for _ in range(50):
            value = _add_pair(value, 1.0)
        return value

    def python_arithmetic():
        value = 0
        for index in range(2000):
            value += index % 7
        return value

    cases = (
        ("python: 2000 int ops, one frame", python_arithmetic),
        ("python: 50 plain calls", python_call_chain),
        ("np.array(6 floats, copy)", lambda: np.array(small, dtype=float, copy=True)),
        ("np.array(6 floats, no copy)", lambda: np.array(small, dtype=float)),
        ("np.asarray(6 floats)", lambda: np.asarray(small, dtype=float)),
        ("np.empty(4)", lambda: np.empty(4)),
        ("np.all(np.isfinite(6))", lambda: np.all(np.isfinite(small))),
        ("np.linalg.norm(4)", lambda: np.linalg.norm(small[:4])),
        ("np.dot(3, 3)", lambda: np.dot(small[1:4], small[1:4])),
        ("np.cross(3, 3)", lambda: np.cross(small[1:4], small[1:4])),
        ("np.concatenate(1+3+4+3)", lambda: np.concatenate((small[:1], small[:3], small[:4], small[1:4]))),
        ("np.r_[0.0, 3 floats]", lambda: np.r_[0.0, small[1:4]]),
        ("np.matmul(4x4, 4x4)", lambda: matrix @ matrix),
    )
    print(f"\n[primitive cost, n={repeat} each]")
    cpu_before, wall_before, ticks_before = _cpu_sample()
    for label, call in cases:
        call()
        samples = []
        for _ in range(repeat):
            start = time.perf_counter()
            call()
            samples.append(time.perf_counter() - start)
        _summary(f"  {label}", samples)
    cpu_after, wall_after, ticks_after = _cpu_sample()
    cpu_total = cpu_after - cpu_before
    wall_total = wall_after - wall_before
    ratio = cpu_total / wall_total if wall_total > 0 else float("nan")
    print(
        f"\n[execution health over the whole run] cpu={cpu_total:.3f}s wall={wall_total:.3f}s " f"cpu/wall={ratio:.2f}"
    )
    if not np.isnan(ticks_before) and ticks_before > 0:
        drift = (ticks_after - ticks_before) - wall_total
        print(
            f"  /proc/uptime advanced {ticks_after - ticks_before:.3f}s over wall {wall_total:.3f}s (drift {drift:+.3f}s)"
        )
    print("  cpu/wall near 1.00 -> the process gets a full core; well below 1.00 -> throttled or descheduled")
    print("\n[reference: the same script on a Windows dev PC, numpy 2.5.1]")
    print("  array-copy 0.9us  asarray 0.3us  empty 0.6us  isfinite 1.3us  norm(4) 1.3us")
    print("  dot 0.7us  cross 10us  concatenate 0.7us  np.r_ 2.0us  matmul(4x4) 0.9us")
    print("  -> a container row 10x-30x above these means per-numpy-operation cost dominates")
    return 0


def _full_cycle(controller, feedback, repeat: int, target_step_mm: float) -> int:
    """Time the whole guarded per-cycle compute path, not just DampedIK.step.

    The 20 ms budget applies to everything the cycle does between the XR sample and
    the joint target submission: feedback re-validation, the DLS step, the frame
    match, the bounds/velocity checks and the repeated path verdict. Measuring the
    step alone can report "fits" while the cycle still overruns.

    The live ``_send_motion`` path is exercised with the arm's wire calls disabled
    (``dry_run`` is already true) and with UDP feedback pinned to the measured
    frame, so no packet is sent and no controller configuration is written.
    """
    dls = controller.dls
    frames = dls.kinematics
    seed = np.array(feedback.joint_rad, dtype=float, copy=True)
    target_xyz, target_quat = frames.forward(seed)
    command = target_xyz + np.array([target_step_mm / 1000.0, 0.0, 0.0])

    inner_step = dls.step
    inner_path = dls.path_allowed
    totals = {"step": 0.0, "path": 0.0}
    step_calls = [0]
    path_calls = [0]
    step_samples = []

    def timed_step(*args, **kwargs):
        start = time.perf_counter()
        result = inner_step(*args, **kwargs)
        elapsed = time.perf_counter() - start
        totals["step"] += elapsed
        step_samples.append(elapsed)
        step_calls[0] += 1
        return result

    def timed_path(*args, **kwargs):
        start = time.perf_counter()
        result = inner_path(*args, **kwargs)
        totals["path"] += time.perf_counter() - start
        path_calls[0] += 1
        return result

    pinned = feedback

    class PinnedReceiver:
        """Re-stamp the measured frame each read so freshness checks see a live arm."""

        thread_error = None

        def latest(self):
            return ArmFeedback(
                received_monotonic_s=time.monotonic(),
                joint_rad=pinned.joint_rad,
                tcp_xyz_m=pinned.tcp_xyz_m,
                tcp_quat_wxyz=pinned.tcp_quat_wxyz,
                arm_error_codes=pinned.arm_error_codes,
                joint_speed_rad_s=pinned.joint_speed_rad_s,
            )

        def stop(self):
            return None

        def start(self):
            return None

        def _store_latest(self, value):
            return None

    original_receiver = controller.feedback_receiver
    controller.feedback_receiver = PinnedReceiver()
    controller.dry_run = True
    dls.step = timed_step
    dls.path_allowed = timed_path

    samples = []
    accepted = []
    try:
        for _ in range(repeat):
            # A segment start: seed from the measured joints, no cached verdicts.
            dls.begin_cycle()
            controller._last_accepted_joint_rad = None
            controller._dls_stopping = True
            controller._rearm_required = False
            controller.arm.fault_latched = False
            start = time.perf_counter()
            accepted.append(bool(controller._send_motion(command, target_quat, seed)))
            samples.append(time.perf_counter() - start)
    finally:
        dls.step = inner_step
        dls.path_allowed = inner_path
        controller.feedback_receiver = original_receiver

    cycle = _summary("  [full cycle] _send_motion (guarded, dry-run)", samples)
    if step_samples:
        _summary("  [full cycle] DampedIK.step inside cycle", step_samples)
    print(
        f"    DampedIK.step aggregate: {totals['step'] * 1000:.1f}ms over {step_calls[0]} calls; "
        f"legacy path_allowed calls={path_calls[0]}\n"
        f"    budget {controller.dt * 1000:.1f}ms -> cycle is "
        f"{cycle / controller.dt:.2f}x the budget"
    )
    if not any(accepted):
        print(
            "    NOTE: no cycle was accepted (motion held or faulted). These timings are an\n"
            "    upper bound on the admitted path, not a completed send."
        )
    return 0


def _xr_calls(repeat: int) -> int:
    """Time the XR SDK calls the teleop cycle makes every iteration.

    These are the only per-cycle costs never measured, and they run inside the same
    20 ms budget as the DLS step. This is a bare measurement: no pose is used for
    motion, no anchor is set, nothing is sent.
    """
    print("\n[XR SDK per-cycle calls]")
    try:
        from xrobotoolkit_teleop.common.xr_client import XrClient

        client = XrClient()
    except Exception as exc:
        print(f"  XR SDK unavailable ({type(exc).__name__}: {exc}); skipping")
        return 0

    calls = (
        ("get_key_value_by_name('right_grip')", lambda: client.get_key_value_by_name("right_grip")),
        ("get_button_state_by_name('B')", lambda: client.get_button_state_by_name("B")),
        ("get_timestamp_ns()", lambda: client.get_timestamp_ns()),
        ("get_pose_by_name('right_controller')", lambda: client.get_pose_by_name("right_controller")),
    )
    total = 0.0
    for label, call in calls:
        try:
            median = _summary(f"  {label}", _time_calls(repeat, call))
        except Exception as exc:
            print(f"  {label}: failed ({type(exc).__name__}: {exc})")
            continue
        total += median
    print(f"  per-cycle XR total: {total * 1000:.3f}ms of a 20.0ms budget")
    return 0


class FakeAlgo:
    """Deterministic stand-in for the SDK FK, used by ``--self-test`` only."""

    def forward(self, joint_rad):
        joints = np.deg2rad(np.asarray(joint_rad, dtype=float))
        xyz = np.array([0.5 * np.cos(joints[0]), 0.5 * np.sin(joints[0]), 0.7 + 0.02 * joints[1]])
        quat = np.array([1.0, 1e-4 * joints[2], 1e-4 * joints[3], 1e-4 * joints[4]])
        return xyz, quat / np.linalg.norm(quat)


def run_self_test(repeat_fk: int, repeat_cycle: int) -> int:
    """Exercise the measurement and reporting path without an arm or the SDK."""
    frames = ControllerFrameKinematics(FakeAlgo(), None, [0.01, 0.02, 0.16, 1.0, 0.0, 0.0, 0.0])
    controller = SimpleNamespace(
        dls=DampedIK(frames),
        dt=0.02,
        max_joint_speed_ratio=0.10,
        max_linear_velocity_m_s=0.05,
        max_angular_velocity_rad_s=0.25,
        _dls_velocity=np.zeros(6),
        feedback_receiver=SimpleNamespace(
            latest=lambda: None,
            stop=lambda: None,
            start=lambda: None,
            _store_latest=lambda value: None,
            valid_packet_count=0,
        ),
    )
    frames.official = SdkTimeProbe(frames.official)

    seed = np.deg2rad(np.array([19.6, -31.5, 55.6, -12.3, -59.3, -98.9]))
    position, orientation = frames.forward(seed)
    feedback = SimpleNamespace(
        joint_rad=seed,
        tcp_xyz_m=position,
        tcp_quat_wxyz=orientation,
        received_monotonic_s=time.monotonic(),
        joint_speed_rad_s=np.zeros(6),
        arm_error_codes=(),
    )
    medians: dict = {}
    shared: dict = {}
    _measure(
        controller,
        seed,
        feedback,
        repeat_fk=repeat_fk,
        repeat_cycle=repeat_cycle,
        target_step_mm=5.0,
        pause_udp=True,
        gc_off_adapter=True,
        medians=medians,
        shared=shared,
    )
    # Four adapter flavours in the three thread states (12) plus the fourth [gc off]
    # state (4) = 16 measured rows.
    expected_rows = 4 * 4
    assert len(medians) == expected_rows, f"expected {expected_rows} rows, got {len(medians)}"
    assert len(shared) >= 3, f"expected attributed rows, got {len(shared)}"
    print(f"\n[self-test] OK: {len(medians)} rows, {len(shared)} attributed rows")

    # Exercise the full-cycle path as well: it is what the 20 ms budget actually
    # applies to, so it must not crash before the board ever runs it. The FMSDK is
    # absent on a dev PC, so the controller is built with an injected fake FK.
    patched_module = sys.modules["xrobotoolkit_teleop.hardware.realman_rm65_safe_teleop_controller"]
    original_ik_class = patched_module.RealmanOfficialRemoteIK
    patched_module.RealmanOfficialRemoteIK = lambda *args, **kwargs: FakeAlgo()
    try:
        controller = RealmanRM65SafeTeleopController(arm=ReadOnlyArm(host="127.0.0.1"), xr=object(), dry_run=True)
    finally:
        patched_module.RealmanOfficialRemoteIK = original_ik_class
    controller_dls = controller.dls
    controller_frames = controller_dls.kinematics
    pose_xyz, pose_quat = controller_frames.forward(seed)
    arm_feedback = ArmFeedback(
        received_monotonic_s=time.monotonic(),
        joint_rad=seed,
        tcp_xyz_m=pose_xyz,
        tcp_quat_wxyz=pose_quat,
        arm_error_codes=(),
        joint_speed_rad_s=np.zeros(6),
    )
    _full_cycle(controller, arm_feedback, repeat=2, target_step_mm=5.0)
    print("[self-test] full-cycle path ran without starting any wire traffic")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-host", default="192.168.10.18")
    parser.add_argument("--repeat-fk", type=int, default=200)
    parser.add_argument("--repeat-cycle", type=int, default=20)
    parser.add_argument("--target-step-mm", type=float, default=5.0)
    parser.add_argument(
        "--stop-udp-during-step",
        action="store_true",
        help="also repeat the whole measurement set with the feedback thread paused",
    )
    parser.add_argument(
        "--gc-off-adapter",
        action="store_true",
        help="repeat the adapter rows with the cyclic collector disabled",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run the measurement path against fakes; no arm, no SDK, no network",
    )
    parser.add_argument(
        "--micro",
        action="store_true",
        help="only time primitive numpy operations; no arm, no SDK, no network",
    )
    parser.add_argument(
        "--full-cycle",
        action="store_true",
        help="also time the whole guarded per-cycle compute path, not just the DLS step",
    )
    parser.add_argument(
        "--xr-calls",
        action="store_true",
        help="only time the per-cycle XR SDK calls; no motion, no anchor",
    )
    args = parser.parse_args()

    if args.micro:
        return _micro(args.repeat_fk)

    if args.xr_calls:
        return _xr_calls(args.repeat_fk)

    if args.self_test:
        return run_self_test(repeat_fk=min(args.repeat_fk, 50), repeat_cycle=min(args.repeat_cycle, 5))

    arm = ReadOnlyArm(host=args.arm_host)
    controller = RealmanRM65SafeTeleopController(arm=arm, xr=object(), dry_run=True)
    try:
        controller._startup()
        receiver = controller.feedback_receiver
        deadline = time.monotonic() + 2.0
        feedback = None
        while feedback is None and time.monotonic() < deadline:
            feedback = receiver.latest()
            time.sleep(0.005)
        if feedback is None:
            raise RuntimeError("no UDP feedback available for the benchmark")
        seed = np.array(feedback.joint_rad, dtype=float, copy=True)
        controller.dls.kinematics.official = SdkTimeProbe(controller.dls.kinematics.official)
        medians: dict = {}
        shared: dict = {}
        _measure(
            controller,
            seed,
            feedback,
            repeat_fk=args.repeat_fk,
            repeat_cycle=args.repeat_cycle,
            target_step_mm=args.target_step_mm,
            pause_udp=args.stop_udp_during_step,
            gc_off_adapter=args.gc_off_adapter,
            medians=medians,
            shared=shared,
        )
        print(
            f"\nbudget: control period={controller.dt * 1000:.1f}ms, "
            f"DLS admission={controller.dls.compute_budget_s * 1000:.1f}ms; "
            f"one miss holds the last accepted target, "
            f"{controller.dls_max_consecutive_failures} consecutive misses slow-stop"
        )
        if args.full_cycle:
            _full_cycle(controller, feedback, repeat=args.repeat_cycle, target_step_mm=args.target_step_mm)
        return 0
    finally:
        try:
            controller.feedback_receiver.stop()
        finally:
            arm.close()


if __name__ == "__main__":
    raise SystemExit(main())
