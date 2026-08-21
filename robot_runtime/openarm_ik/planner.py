"""Outer loop that turns repeated IK steps into a joint-space trajectory."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from robot_runtime.openarm_ik.constraints import IKConstraints
from robot_runtime.openarm_ik.kinematics import fk
from robot_runtime.openarm_ik.model import ORIGIN_FRAME
from robot_runtime.openarm_ik.solver import IKStepConfig, step


@dataclass(frozen=True)
class TrajectoryPoint:
    """One recorded sample of the outer IK loop."""

    t: float
    q: tuple[float, ...]
    ee_pose: tuple[float, ...]


@dataclass(frozen=True)
class ReachPlan:
    """Cartesian reach result in the arm_origin frame."""

    arm: str
    frame: str
    dt: float
    points: tuple[TrajectoryPoint, ...]
    target_pose: tuple[float, ...]
    final_error_m: float
    ok: bool
    failure_reason: str


def plan_reach(
    side: str,
    joints: Sequence[float],
    target: Sequence[float],
    *,
    max_steps: int = 80,
    pos_tol: float = 0.006,
    dt: float = 0.05,
    config: IKConstraints | IKStepConfig | None = None,
) -> ReachPlan:
    """Move from the current joints toward an xyz or pose[7] target.

    The returned trajectory starts at the current configuration. Each later
    point is one ``step()``. Orientation defaults to the current EE quaternion
    when ``target`` is only xyz.
    """

    if max_steps <= 0:
        raise ValueError("max_steps must be positive")
    if pos_tol <= 0.0:
        raise ValueError("pos_tol must be positive")
    if dt <= 0.0:
        raise ValueError("dt must be positive")

    current_q = tuple(float(value) for value in joints)
    current_pose = fk(side, current_q)
    target_pose = _assemble_target(current_pose, target)
    points = [TrajectoryPoint(t=0.0, q=current_q, ee_pose=current_pose)]
    error = _position_error(current_pose, target_pose)
    if error <= pos_tol:
        return _result(side, dt, points, target_pose, error, True, "converged")

    for index in range(1, max_steps + 1):
        current_q = step(side, current_q, target_pose, config=config)
        current_pose = fk(side, current_q)
        points.append(TrajectoryPoint(t=index * dt, q=current_q, ee_pose=current_pose))
        error = _position_error(current_pose, target_pose)
        if error <= pos_tol:
            return _result(side, dt, points, target_pose, error, True, "converged")

    return _result(side, dt, points, target_pose, error, False, "max_steps")


def _assemble_target(
    current_pose: tuple[float, ...],
    target: Sequence[float],
) -> tuple[float, ...]:
    values = tuple(float(value) for value in target)
    if len(values) == 3:
        return values + current_pose[3:]
    if len(values) == 7:
        return values
    raise ValueError("target must be xyz (3) or pose[7]")


def _position_error(pose: tuple[float, ...], target: tuple[float, ...]) -> float:
    return math.sqrt(sum((pose[index] - target[index]) ** 2 for index in range(3)))


def _result(
    side: str,
    dt: float,
    points: list[TrajectoryPoint],
    target_pose: tuple[float, ...],
    error: float,
    ok: bool,
    failure_reason: str,
) -> ReachPlan:
    return ReachPlan(
        arm=side,
        frame=ORIGIN_FRAME,
        dt=dt,
        points=tuple(points),
        target_pose=target_pose,
        final_error_m=error,
        ok=ok,
        failure_reason=failure_reason,
    )
