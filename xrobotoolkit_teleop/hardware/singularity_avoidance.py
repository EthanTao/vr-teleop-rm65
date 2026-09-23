"""Offline structural singularity analysis (not used to authorize hardware motion).

The hardware path uses damped_ik.py with official FK and sends its guarded joint step.

This historical structural model only estimates three quantities for offline analysis:

1. how close is this joint configuration to a kinematic singularity (``severity``, ``zone``);
2. how fast may the operator-commanded motion still be executed (``speed_scale``);
3. given the commanded Cartesian motion, which joint step would a damped least-squares (DLS)
   inverse produce, and is that step heading further into the singularity (``hold``).

Kinematic model
---------------
``RM65_CHAIN`` reproduces the vendor's own joint chain from
``assets/realman/RM65-official/urdf/RM65-official-arm.urdf``: every joint has a fixed origin
transform in its parent link and rotates about the local ``z`` axis. The link magnitudes match
the official ontology page (``docs/RM65参数与坐标系.md`` section 3: ``d1 = 240.5 mm``,
``a2 = 256 mm``, ``d4 = 210 mm``) and the last-link offset ``d6 = 112.03 mm`` that the page
leaves model-dependent is taken from that URDF. ``tests/test_singularity_avoidance.py``
re-derives both the constants and the Jacobian from the URDF file, so a transcription error
cannot pass unnoticed.

This local model does not reproduce the controller TCP pose, so absolute poses from
:func:`rm65_forward_kinematics` must never be compared with real TCP feedback. Its historical decision quantities are: the singular-value ratio ``sigma_min / sigma_max``
(reciprocal condition number) and the official ``J3``/``J5`` near-zero criteria.

Decision quantities
-------------------
* ``severity`` in ``[0, 1]``: 0 when clear, 1 at or beyond the stop threshold. It is the
  maximum of a Jacobian-ratio ramp and the official joint-angle ramps.
* ``score`` = ``-log10(sigma_ratio)``: unlike the raw ratio it keeps resolution deep inside the
  danger zone, so "moving further in" stays detectable next to a singularity. It is saturated
  at ``_RATIO_FLOOR`` so a numerically singular configuration cannot look "deeper" than another.
* ``speed_scale``: how much of the commanded velocity may still be executed (never above 1).
* ``hold``: the command would move deeper into the danger zone, as an offline classification only (freezing a target does not stop a moving arm).
  It is decided from a bounded joint-space probe along the command direction, never from the
  damped step magnitude, and does not imply that any real robot motion is permitted.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

__all__ = [
    "RM65_CHAIN",
    "CommandGate",
    "JointChain",
    "SingularityGuard",
    "SingularityMeasure",
    "SingularityState",
    "rm65_forward_kinematics",
    "rm65_geometric_jacobian",
]


_RATIO_FLOOR = 1e-9
"""Singular-value ratios below this value are treated as numerically singular."""


@dataclass(frozen=True)
class JointChain:
    """Serial chain with a fixed origin transform per joint and rotation about local ``z``.

    ``origin_xyz_m`` and ``origin_rpy_rad`` are expressed in the parent link frame, exactly as
    the ``<origin>`` element of a URDF joint; ``rpy`` is applied as ``Rz(yaw) Ry(pitch) Rx(roll)``.
    """

    origin_xyz_m: np.ndarray
    origin_rpy_rad: np.ndarray

    def __post_init__(self) -> None:
        for name in ("origin_xyz_m", "origin_rpy_rad"):
            value = np.array(getattr(self, name), dtype=float, copy=True)
            if value.shape != (6, 3):
                raise ValueError(f"{name} must have shape (6, 3)")
            if not np.all(np.isfinite(value)):
                raise ValueError(f"{name} must contain only finite values")
            value.setflags(write=False)
            object.__setattr__(self, name, value)


RM65_CHAIN = JointChain(
    origin_xyz_m=np.array(
        [
            [0.0, 0.0, 0.2405],
            [0.0, 0.0, 0.0],
            [0.256, 0.0, 0.0],
            [0.0, -0.21, -0.00030007],
            [0.0, 0.0, 0.0],
            [0.0, -0.11203, -0.00029971],
        ]
    ),
    origin_rpy_rad=np.deg2rad(
        np.array(
            [
                [0.0, 0.0, -180.0],
                [90.0, -90.0, 0.0],
                [0.0, 0.0, 90.0],
                [90.0, 0.0, 0.0],
                [-90.0, 0.0, 0.0],
                [90.0, 0.0, 0.0],
            ]
        )
    ),
)
"""Vendor joint chain of the official RM65 URDF (see the module docstring)."""


def _finite_joints(joint_rad: np.ndarray) -> np.ndarray:
    try:
        joints = np.array(joint_rad, dtype=float, copy=True)
    except (TypeError, ValueError) as exc:
        raise ValueError("joint_rad must be a finite numeric array with shape (6,)") from exc
    if joints.shape != (6,):
        raise ValueError("joint_rad must have shape (6,)")
    if not np.all(np.isfinite(joints)):
        raise ValueError("joint_rad must contain only finite values")
    return joints


def _rpy_transform(rpy_rad: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy_rad)
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    rotate_x = np.array([[1.0, 0.0, 0.0], [0.0, cos_r, -sin_r], [0.0, sin_r, cos_r]])
    rotate_y = np.array([[cos_p, 0.0, sin_p], [0.0, 1.0, 0.0], [-sin_p, 0.0, cos_p]])
    rotate_z = np.array([[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]])
    transform = np.eye(4)
    transform[:3, :3] = rotate_z @ rotate_y @ rotate_x
    return transform


def _origin_transform(chain: JointChain, index: int) -> np.ndarray:
    transform = _rpy_transform(chain.origin_rpy_rad[index])
    transform[:3, 3] = chain.origin_xyz_m[index]
    return transform


def _joint_rotation(angle: float) -> np.ndarray:
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    return np.array(
        [
            [cos_a, -sin_a, 0.0, 0.0],
            [sin_a, cos_a, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def _frame_chain(joint_rad: np.ndarray, chain: JointChain) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return joint-frame origins ``(6, 3)``, joint axes ``(6, 3)`` and the tip transform."""
    joints = _finite_joints(joint_rad)
    origins = np.zeros((6, 3), dtype=float)
    axes = np.zeros((6, 3), dtype=float)
    transform = np.eye(4)
    for index in range(6):
        joint_frame = transform @ _origin_transform(chain, index)
        origins[index] = joint_frame[:3, 3]
        axes[index] = joint_frame[:3, 2]
        transform = joint_frame @ _joint_rotation(float(joints[index]))
    return origins, axes, transform


def rm65_forward_kinematics(joint_rad: np.ndarray, chain: JointChain = RM65_CHAIN) -> np.ndarray:
    """Return the structural flange transform for ``joint_rad`` as a ``(4, 4)`` matrix.

    This is a local structural model for manipulability only, not the controller TCP pose.
    """
    return _frame_chain(joint_rad, chain)[2]


def rm65_geometric_jacobian(joint_rad: np.ndarray, chain: JointChain = RM65_CHAIN) -> np.ndarray:
    """Return the base-frame geometric Jacobian ``(6, 6)``: linear rows then angular rows."""
    origins, axes, tip = _frame_chain(joint_rad, chain)
    tip_position = tip[:3, 3]
    jacobian = np.zeros((6, 6), dtype=float)
    for index in range(6):
        jacobian[:3, index] = np.cross(axes[index], tip_position - origins[index])
        jacobian[3:, index] = axes[index]
    return jacobian


@dataclass(frozen=True)
class SingularityMeasure:
    """Model-only proximity numbers; thresholds are applied by :class:`SingularityGuard`."""

    sigma_min: float
    sigma_max: float
    sigma_ratio: float
    score: float
    j3_rad: float
    j5_rad: float
    reasons: tuple[str, ...]


@dataclass(frozen=True)
class SingularityState:
    """Proximity plus the guard thresholds: zone, severity and the allowed speed scale."""

    measure: SingularityMeasure
    severity: float
    zone: str
    speed_scale: float

    @property
    def sigma_min(self) -> float:
        return self.measure.sigma_min

    @property
    def sigma_ratio(self) -> float:
        return self.measure.sigma_ratio

    @property
    def score(self) -> float:
        return self.measure.score

    @property
    def reasons(self) -> tuple[str, ...]:
        return self.measure.reasons

    @property
    def is_clear(self) -> bool:
        return self.zone == "clear"


@dataclass(frozen=True)
class CommandGate:
    """Per-cycle singularity decision for one commanded target."""

    speed_scale: float
    hold: bool
    zone: str
    severity: float
    inward: bool
    damping: float
    step_scale: float
    current: SingularityState
    predicted: SingularityState
    predicted_joint_rad: np.ndarray
    damped_step_rad: np.ndarray

    @property
    def reasons(self) -> tuple[str, ...]:
        merged = list(self.current.reasons)
        merged.extend(reason for reason in self.predicted.reasons if reason not in merged)
        return tuple(merged)

    @property
    def summary(self) -> str:
        state = self.current
        parts = [
            f"zone={self.zone}",
            f"sigma_min={state.sigma_min:.4f}",
            f"ratio={state.sigma_ratio:.4f}",
            f"severity={self.severity:.2f}",
            f"scale={self.speed_scale:.2f}",
        ]
        if self.hold:
            parts.append("HOLD(inward blocked)")
        elif self.inward:
            parts.append("slowing")
        if self.reasons:
            parts.append("reasons=" + ",".join(self.reasons))
        return " ".join(parts)


class SingularityGuard:
    """Bounded, direction-aware singularity slowdown for teleoperation commands.

    The guard is deliberately one-sided: every returned scale is in ``[0, 1]`` and the returned
    ``hold`` flag can only suppress motion that would move deeper into a singularity. Leaving a
    singularity is never blocked, so an operator can always drive out.
    """

    def __init__(
        self,
        *,
        chain: JointChain = RM65_CHAIN,
        slowdown_ratio: float = 0.004,
        stop_ratio: float = 0.0012,
        slowdown_joint_rad: float = math.radians(15.0),
        stop_joint_rad: float = math.radians(5.0),
        min_speed_scale: float = 0.05,
        escape_speed_scale: float = 0.50,
        max_damping: float = 0.08,
        probe_joint_rad: float = math.radians(1.0),
        score_tolerance: float = 0.02,
        severity_tolerance: float = 1e-3,
    ) -> None:
        self.chain = chain
        self.slowdown_ratio = self._unit_interval(slowdown_ratio, "slowdown_ratio")
        self.stop_ratio = self._unit_interval(stop_ratio, "stop_ratio")
        if self.stop_ratio >= self.slowdown_ratio:
            raise ValueError("stop_ratio must be below slowdown_ratio")
        self.slowdown_joint_rad = self._positive(slowdown_joint_rad, "slowdown_joint_rad")
        self.stop_joint_rad = self._non_negative(stop_joint_rad, "stop_joint_rad")
        if self.stop_joint_rad >= self.slowdown_joint_rad:
            raise ValueError("stop_joint_rad must be below slowdown_joint_rad")
        self.min_speed_scale = self._positive(min_speed_scale, "min_speed_scale")
        if self.min_speed_scale > 1.0:
            raise ValueError("min_speed_scale must not exceed 1.0")
        self.escape_speed_scale = self._positive(escape_speed_scale, "escape_speed_scale")
        if self.escape_speed_scale > 1.0:
            raise ValueError("escape_speed_scale must not exceed 1.0")
        if self.min_speed_scale > self.escape_speed_scale:
            raise ValueError("min_speed_scale must not exceed escape_speed_scale")
        self.max_damping = self._positive(max_damping, "max_damping")
        self.probe_joint_rad = self._positive(probe_joint_rad, "probe_joint_rad")
        self.score_tolerance = self._non_negative(score_tolerance, "score_tolerance")
        self.severity_tolerance = self._non_negative(severity_tolerance, "severity_tolerance")

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
    def _positive(cls, value: float, name: str) -> float:
        result = cls._finite(value, name)
        if result <= 0.0:
            raise ValueError(f"{name} must be positive")
        return result

    @classmethod
    def _non_negative(cls, value: float, name: str) -> float:
        result = cls._finite(value, name)
        if result < 0.0:
            raise ValueError(f"{name} must be non-negative")
        return result

    @classmethod
    def _unit_interval(cls, value: float, name: str) -> float:
        result = cls._positive(value, name)
        if result >= 1.0:
            raise ValueError(f"{name} must be below 1.0")
        return result

    @staticmethod
    def _ramp(value: float, warning: float, stop: float) -> float:
        """Return 0 at or above ``warning``, 1 at or below ``stop``, linear in between."""
        if value >= warning:
            return 0.0
        if value <= stop:
            return 1.0
        return float((warning - value) / (warning - stop))

    def measure(self, joint_rad: np.ndarray) -> SingularityMeasure:
        """Return the local proximity numbers for one joint configuration."""
        joints = _finite_joints(joint_rad)
        singular_values = np.linalg.svd(rm65_geometric_jacobian(joints, self.chain), compute_uv=False)
        sigma_max = float(singular_values[0])
        sigma_min = float(singular_values[-1])
        ratio = sigma_min / sigma_max if sigma_max > 0.0 else 0.0
        ratio = max(ratio, _RATIO_FLOOR)
        j3 = float(joints[2])
        j5 = float(joints[4])
        reasons = []
        if abs(j3) <= self.slowdown_joint_rad:
            reasons.append(f"elbow J3={math.degrees(j3):.1f}deg")
        if abs(j5) <= self.slowdown_joint_rad:
            reasons.append(f"wrist J5={math.degrees(j5):.1f}deg")
        if ratio <= self.slowdown_ratio:
            reasons.append(f"jacobian ratio={ratio:.4f}")
        return SingularityMeasure(
            sigma_min=sigma_min,
            sigma_max=sigma_max,
            sigma_ratio=ratio,
            score=float(-math.log10(ratio)),
            j3_rad=j3,
            j5_rad=j5,
            reasons=tuple(reasons),
        )

    def state(self, joint_rad: np.ndarray) -> SingularityState:
        """Return proximity plus the configured zone, severity and speed scale."""
        measure = self.measure(joint_rad)
        severity = max(
            self._ramp(measure.sigma_ratio, self.slowdown_ratio, self.stop_ratio),
            self._ramp(abs(measure.j3_rad), self.slowdown_joint_rad, self.stop_joint_rad),
            self._ramp(abs(measure.j5_rad), self.slowdown_joint_rad, self.stop_joint_rad),
        )
        if severity <= 0.0:
            zone = "clear"
        elif severity >= 1.0:
            zone = "danger"
        else:
            zone = "slowdown"
        return SingularityState(
            measure=measure,
            severity=severity,
            zone=zone,
            speed_scale=1.0 - severity * (1.0 - self.min_speed_scale),
        )

    def damping_for(self, severity: float) -> float:
        """Return the DLS damping factor: zero when clear, ``max_damping`` at the stop threshold."""
        value = min(1.0, max(0.0, self._finite(severity, "severity")))
        return self.max_damping * value

    def damped_joint_step(
        self,
        joint_rad: np.ndarray,
        twist: np.ndarray,
        damping: float | None = None,
    ) -> np.ndarray:
        """Return the damped least-squares joint step for one Cartesian twist.

        ``twist`` is ``[linear_xyz, angular_xyz]`` in the robot base frame. Damping defaults to
        the value implied by the current configuration, so the step stays bounded near a
        singularity instead of diverging like a plain pseudo-inverse.
        """
        joints = _finite_joints(joint_rad)
        try:
            velocity = np.array(twist, dtype=float, copy=True)
        except (TypeError, ValueError) as exc:
            raise ValueError("twist must be a finite numeric array with shape (6,)") from exc
        if velocity.shape != (6,) or not np.all(np.isfinite(velocity)):
            raise ValueError("twist must be a finite numeric array with shape (6,)")
        if damping is None:
            factor = self.damping_for(self.state(joints).severity)
        else:
            factor = self._non_negative(damping, "damping")
        jacobian = rm65_geometric_jacobian(joints, self.chain)
        regularized = jacobian @ jacobian.T + (factor * factor + 1e-12) * np.eye(6)
        return jacobian.T @ np.linalg.solve(regularized, velocity)

    def joint_step_scale(self, step_rad: np.ndarray, budget_rad: np.ndarray) -> float:
        """Return a scale in ``(0, 1]`` keeping every joint step inside its per-cycle budget."""
        step = np.asarray(step_rad, dtype=float)
        budget = np.asarray(budget_rad, dtype=float)
        if step.shape != (6,) or not np.all(np.isfinite(step)):
            raise ValueError("step_rad must be a finite array with shape (6,)")
        if budget.shape != (6,) or not np.all(np.isfinite(budget)) or np.any(budget <= 0.0):
            raise ValueError("budget_rad must be a positive finite array with shape (6,)")
        peak = float(np.max(np.abs(step) / budget))
        return 1.0 if peak <= 1.0 else 1.0 / peak

    def _gate(
        self,
        current: SingularityState,
        predicted: SingularityState,
        predicted_joint_rad: np.ndarray,
        damping: float,
        step_scale: float,
        damped_step_rad: np.ndarray,
    ) -> CommandGate:
        inward = predicted.score > current.score + 1e-10 or (predicted.severity > current.severity + 1e-10)
        hold = current.zone == "danger" and inward
        if inward:
            # Approaching: take the more conservative of both ends of the command.
            approach_scale = min(current.speed_scale, predicted.speed_scale)
        else:
            # Stationary or leaving: never throttle below the escape scale, so the operator
            # can always drive back out of the danger zone.
            approach_scale = max(current.speed_scale, self.escape_speed_scale)
        # Direction-independent joint feasibility cap: a command that would need far more joint
        # motion than one cycle allows is slowed even when it is not approaching a singularity.
        speed_scale = min(approach_scale, step_scale)
        if current.is_clear and predicted.is_clear:
            # "Normal motion": outside every ramp the guard must not change teleoperation at all.
            # The pre-existing joint-overspeed watchdog stays the protection out there.
            speed_scale = 1.0
        if hold:
            # Block the step into the singularity; the caller freezes its target so the
            # existing acceleration limit still produces a smooth stop.
            speed_scale = self.min_speed_scale
        return CommandGate(
            speed_scale=float(speed_scale),
            hold=bool(hold),
            zone=current.zone,
            severity=current.severity,
            inward=bool(inward),
            damping=float(damping),
            step_scale=float(step_scale),
            current=current,
            predicted=predicted,
            predicted_joint_rad=np.array(predicted_joint_rad, dtype=float, copy=True),
            damped_step_rad=np.array(damped_step_rad, dtype=float, copy=True),
        )

    def _probe_state(self, joints: np.ndarray, twist: np.ndarray) -> tuple[SingularityState, np.ndarray]:
        """Return the state reached by a bounded joint-space probe along the command direction.

        The damped step keeps a near-singular command bounded, but it also suppresses the very
        direction that leads into the singularity, which would hide "still moving in". The probe
        therefore normalizes the *undamped* least-squares direction to ``probe_joint_rad``: the
        direction answers "does this command deepen the singularity", the magnitude is irrelevant.
        """
        direction = self.damped_joint_step(joints, twist, 0.0)
        norm = float(np.linalg.norm(direction))
        if not math.isfinite(norm) or norm <= 0.0:
            return self.state(joints), joints.copy()
        # Never normalize a tiny request into a one-degree crossing.
        distance = min(norm, self.probe_joint_rad, math.radians(0.01))
        for index in (2, 4):
            if joints[index] * direction[index] < 0:
                distance = min(distance, abs(joints[index]) * norm / abs(direction[index]) * 0.25)
        probed = joints + direction * (distance / norm)
        return self.state(probed), probed

    def command_gate(
        self,
        current_joint_rad: np.ndarray,
        twist: np.ndarray,
        budget_rad: np.ndarray,
    ) -> CommandGate:
        """Return the singularity decision for a Cartesian command step.

        The requested twist is turned into a damped least-squares joint step (bounded motion) and
        into a bounded direction probe (is this command heading further in). ``budget_rad`` is the
        per-joint per-cycle step budget that the hardware path already enforces for solved joint
        targets.
        """
        joints = _finite_joints(current_joint_rad)
        current = self.state(joints)
        damping = self.damping_for(current.severity)
        damped_step = self.damped_joint_step(joints, twist, damping)
        step_scale = self.joint_step_scale(damped_step, budget_rad)
        predicted, probed = self._probe_state(joints, twist)
        return self._gate(current, predicted, probed, damping, step_scale, damped_step)

    def joint_gate(self, current_joint_rad: np.ndarray, target_joint_rad: np.ndarray) -> CommandGate:
        """Return the singularity decision for a solved joint target (official IK path)."""
        joints = _finite_joints(current_joint_rad)
        target = _finite_joints(target_joint_rad)
        current = self.state(joints)
        predicted = self.state(target)
        return self._gate(
            current,
            predicted,
            target,
            self.damping_for(current.severity),
            1.0,
            target - joints,
        )
