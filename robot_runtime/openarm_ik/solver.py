"""One official-style IK solve: mink 7×7 QP on the 6×7 Jacobian.

Each ``step()`` is ``max_iters`` substeps of the OpenArm objective:
weighted frame tracking, LM + Tikhonov damping, kinetic energy
``cost * M / dt²``, 1D nullspace posture, and a linearized singularity
inequality. Error is expressed in the arm_origin frame so it matches
``jacobian()``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from robot_runtime.openarm_ik.constraints import (
    IKConstraints,
    OFFICIAL_CONSTRAINTS,
    apply_joint_limits,
    arm_mass_matrix,
    bound_frame_error,
    clip_joint_velocity,
    nullspace_fade,
    singularity_approach_rate,
    singularity_ratio,
    substep_dt,
    weight_task,
)
from robot_runtime.openarm_ik.kinematics import fk, jacobian
from robot_runtime.openarm_ik.model import arm


@dataclass(frozen=True)
class IKStepConfig:
    """Subset of IKConstraints kept so existing ``plan_reach(..., config=)`` calls work.

    Prefer ``IKConstraints`` when you want the full official set. Passing this
    object still loads the other official numbers (costs, singularity, braking).
    """

    damping: float = 0.01
    position_error_limit: float = 0.02
    orientation_error_limit: float = 0.25
    nullspace_gain: float = 0.2

    def __post_init__(self) -> None:
        for name in (
            "damping",
            "position_error_limit",
            "orientation_error_limit",
            "nullspace_gain",
        ):
            value = getattr(self, name)
            if value < 0.0:
                raise ValueError(f"{name} must not be negative")

    def to_constraints(self) -> IKConstraints:
        return IKConstraints(
            lm_damping=self.damping,
            frame_position_error_limit=self.position_error_limit,
            frame_orientation_error_limit=self.orientation_error_limit,
            enable_nullspace=self.nullspace_gain > 0.0,
            enable_position_bound=self.position_error_limit > 0.0,
        )


def resolve_constraints(
    config: IKConstraints | IKStepConfig | None,
) -> IKConstraints:
    if config is None:
        return OFFICIAL_CONSTRAINTS
    if isinstance(config, IKConstraints):
        return config
    return config.to_constraints()


def step(
    side: str,
    joints: Sequence[float],
    target: Sequence[float],
    *,
    config: IKConstraints | IKStepConfig | None = None,
) -> tuple[float, ...]:
    """Take one official-style ``solve()`` toward ``target`` pose[7].

    That is ``max_iters`` constrained substeps, not a single linear bite.
    ``joints`` may be 7 arm angles or q8 with a trailing gripper; the gripper
    is copied through unchanged.
    """

    settings = resolve_constraints(config)
    kinematics = arm(side)
    values = tuple(float(value) for value in joints)
    gripper: float | None = None
    if len(values) == kinematics.q8_length:
        gripper = values[-1]
        values = values[: kinematics.n_joints]
    if len(values) != kinematics.n_joints:
        raise ValueError(
            f"expected {kinematics.n_joints} arm joints or {kinematics.q8_length} "
            f"driver values, got {len(values)}"
        )
    goal = tuple(float(value) for value in target)
    if len(goal) != 7:
        raise ValueError("target pose must have 7 values [px, py, pz, qw, qx, qy, qz]")

    for _ in range(settings.max_iters):
        values = _substep(side, values, goal, settings, kinematics)

    if gripper is None:
        return values
    return values + (gripper,)


def _substep(
    side: str,
    values: tuple[float, ...],
    goal: tuple[float, ...],
    settings: IKConstraints,
    kinematics,
) -> tuple[float, ...]:
    error = bound_frame_error(_pose_error(fk(side, values), goal), settings)
    task_jacobian = jacobian(side, values)
    weighted_jacobian, weighted_error = weight_task(task_jacobian, error, settings)
    dt = substep_dt(settings)
    direction = None
    displacement = 0.0
    nullspace_cost = 0.0
    if settings.enable_nullspace and settings.nullspace_cost > 0.0:
        direction = _nullspace_direction(
            task_jacobian, values, kinematics.home, settings.lm_damping
        )
        posture_error = sum(
            axis * (angle - rest)
            for axis, angle, rest in zip(direction, values, kinematics.home)
        )
        speed = -settings.nullspace_return_rate * posture_error
        speed = max(
            -settings.nullspace_max_speed,
            min(settings.nullspace_max_speed, speed),
        )
        fade = nullspace_fade(singularity_ratio(task_jacobian, settings), settings)
        displacement = fade * speed * dt
        nullspace_cost = math.sqrt(max(fade, 0.0)) * settings.nullspace_cost
    hessian, delta = _mink_task_step(
        weighted_jacobian,
        weighted_error,
        settings,
        mass=arm_mass_matrix(side),
        dt=dt,
        nullspace_direction=direction,
        nullspace_displacement=displacement,
        nullspace_cost=nullspace_cost,
    )
    delta = clip_joint_velocity(delta, settings)
    if settings.enable_singularity_limit:
        ratio_now = singularity_ratio(task_jacobian, settings)
        gradient = _singularity_gradient(side, values, settings)
        allowed = singularity_approach_rate(ratio_now, settings) * dt
        delta = _enforce_singularity_plane(hessian, delta, gradient, allowed)
    return apply_joint_limits(values, delta, kinematics.hinges, settings)


def _mink_task_step(
    weighted_jacobian: tuple[tuple[float, ...], ...],
    weighted_error: tuple[float, ...],
    settings: IKConstraints,
    *,
    mass: tuple[tuple[float, ...], ...],
    dt: float,
    nullspace_direction: tuple[float, ...] | None,
    nullspace_displacement: float,
    nullspace_cost: float,
) -> tuple[tuple[tuple[float, ...], ...], tuple[float, ...]]:
    """Assemble mink's 7×7 objective and solve H Δq = Jwᵀ we.

    Matches ``Task._assemble_qp`` plus global damping plus
    ``cost * M / dt²`` kinetic regularization, with the 1D nullspace
    task added the same way (H += c² nnᵀ, rhs += c² disp n).
    """

    joint_count = len(weighted_jacobian[0])
    mu = settings.lm_damping * sum(value * value for value in weighted_error)
    kinetic_scale = 0.0
    if settings.kinetic_energy_cost > 0.0 and dt > 0.0:
        kinetic_scale = settings.kinetic_energy_cost / (dt * dt)
    hessian = []
    rhs = []
    for row in range(joint_count):
        force = sum(
            weighted_jacobian[task][row] * weighted_error[task] for task in range(6)
        )
        line = []
        for col in range(joint_count):
            value = sum(
                weighted_jacobian[task][row] * weighted_jacobian[task][col]
                for task in range(6)
            )
            if row == col:
                value += mu + settings.damping
            if kinetic_scale > 0.0:
                value += kinetic_scale * mass[row][col]
            line.append(value)
        hessian.append(line)
        rhs.append(force)
    if (
        nullspace_direction is not None
        and nullspace_cost > 0.0
    ):
        cost_sq = nullspace_cost * nullspace_cost
        for row in range(joint_count):
            rhs[row] += cost_sq * nullspace_displacement * nullspace_direction[row]
            for col in range(joint_count):
                hessian[row][col] += (
                    cost_sq * nullspace_direction[row] * nullspace_direction[col]
                )
    packed = tuple(tuple(line) for line in hessian)
    return packed, _solve_linear_system(packed, tuple(rhs))


def _enforce_singularity_plane(
    hessian: tuple[tuple[float, ...], ...],
    delta: tuple[float, ...],
    gradient: tuple[float, ...],
    allowed: float,
) -> tuple[float, ...]:
    """Project Δq onto -∇ρ · Δq ≤ allowed in the QP metric H.

    Uniform scaling would shrink the whole reach; the official DAQP
    inequality only removes the component that dives into a singularity.
    """

    decrease = -sum(grad * step for grad, step in zip(gradient, delta))
    if decrease <= allowed:
        return delta
    correction = _solve_linear_system(hessian, gradient)
    denom = sum(grad * value for grad, value in zip(gradient, correction))
    if abs(denom) < 1e-18:
        return delta
    lagrange = (decrease - allowed) / denom
    return tuple(step + lagrange * value for step, value in zip(delta, correction))


def _singularity_gradient(
    side: str,
    values: tuple[float, ...],
    settings: IKConstraints,
) -> tuple[float, ...]:
    """Central-difference ∇ρ on the seven arm joints, matching official epsilon."""

    eps = settings.singularity_gradient_epsilon
    gradient = []
    for index in range(len(values)):
        plus = list(values)
        minus = list(values)
        plus[index] += eps
        minus[index] -= eps
        ratio_plus = singularity_ratio(jacobian(side, plus), settings)
        ratio_minus = singularity_ratio(jacobian(side, minus), settings)
        gradient.append((ratio_plus - ratio_minus) / (2.0 * eps))
    return tuple(gradient)


def _nullspace_direction(
    task_jacobian: tuple[tuple[float, ...], ...],
    values: tuple[float, ...],
    home: tuple[float, ...],
    damping: float,
) -> tuple[float, ...]:
    seed = tuple(rest - angle for rest, angle in zip(home, values))
    if sum(component * component for component in seed) < 1e-18:
        seed = (0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    vector = seed
    for _ in range(6):
        vector = _nullspace_project(task_jacobian, vector, damping)
        norm = math.sqrt(sum(component * component for component in vector))
        if norm < 1e-18:
            return seed
        vector = tuple(component / norm for component in vector)
    return vector


def _pose_error(current: tuple[float, ...], target: tuple[float, ...]) -> tuple[float, ...]:
    linear = (
        target[0] - current[0],
        target[1] - current[1],
        target[2] - current[2],
    )
    relative = _matmul3(
        _quat_to_rotation(target[3:]),
        _transpose3(_quat_to_rotation(current[3:])),
    )
    return linear + _log_so3(relative)


def _damped_task_step(
    task_jacobian: tuple[tuple[float, ...], ...],
    error: tuple[float, ...],
    damping: float,
) -> tuple[float, ...]:
    system = _damped_task_matrix(task_jacobian, damping)
    task_velocity = _solve_linear_system(system, error)
    joint_count = len(task_jacobian[0])
    return tuple(
        sum(task_jacobian[row][joint] * task_velocity[row] for row in range(6))
        for joint in range(joint_count)
    )


def _nullspace_project(
    task_jacobian: tuple[tuple[float, ...], ...],
    secondary: tuple[float, ...],
    damping: float,
) -> tuple[float, ...]:
    task_motion = tuple(
        sum(task_jacobian[row][joint] * secondary[joint] for joint in range(len(secondary)))
        for row in range(6)
    )
    removed = _damped_task_step(task_jacobian, task_motion, damping)
    return tuple(value - correction for value, correction in zip(secondary, removed))


def _damped_task_matrix(
    task_jacobian: tuple[tuple[float, ...], ...],
    damping: float,
) -> tuple[tuple[float, ...], ...]:
    rows = []
    for row in range(6):
        line = []
        for col in range(6):
            value = sum(
                task_jacobian[row][joint] * task_jacobian[col][joint]
                for joint in range(len(task_jacobian[0]))
            )
            if row == col:
                value += damping
            line.append(value)
        rows.append(tuple(line))
    return tuple(rows)


def _solve_linear_system(
    matrix: tuple[tuple[float, ...], ...],
    rhs: tuple[float, ...],
) -> tuple[float, ...]:
    size = len(rhs)
    augmented = [list(matrix[row]) + [rhs[row]] for row in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            raise ValueError("damped IK system is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        divisor = augmented[column][column]
        for index in range(column, size + 1):
            augmented[column][index] /= divisor
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            for index in range(column, size + 1):
                augmented[row][index] -= factor * augmented[column][index]
    return tuple(augmented[row][size] for row in range(size))


def _quat_to_rotation(quat: tuple[float, ...]) -> tuple[tuple[float, ...], ...]:
    w, x, y, z = quat
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def _transpose3(
    matrix: tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(matrix[row][col] for row in range(3)) for col in range(3))


def _matmul3(
    left: tuple[tuple[float, ...], ...],
    right: tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(left[row][k] * right[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def _log_so3(rotation: tuple[tuple[float, ...], ...]) -> tuple[float, float, float]:
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    cosine = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
    angle = math.acos(cosine)
    if angle < 1e-8:
        return (
            0.5 * (rotation[2][1] - rotation[1][2]),
            0.5 * (rotation[0][2] - rotation[2][0]),
            0.5 * (rotation[1][0] - rotation[0][1]),
        )
    scale = angle / (2.0 * math.sin(angle))
    return (
        scale * (rotation[2][1] - rotation[1][2]),
        scale * (rotation[0][2] - rotation[2][0]),
        scale * (rotation[1][0] - rotation[0][1]),
    )
