"""Official OpenArm IK constraint numbers, stored as pure math.

Source of the numbers is ``openarm_control`` ``IKParams`` plus the QP
pieces that mink assembled around it:

* ``BoundedFrameTask`` — clip the Cartesian bite this substep may take
* ``FrameTask`` costs — weight position vs orientation
* ``NullspacePostureTask`` — 7-DoF leftover pulls toward home
* ``SingularityApproachLimit`` — do not dive into a flattened Jacobian
* ``ArmConfigurationLimit`` — hinge stops with gain 0.95
* ``KineticEnergyRegularizationTask`` — ``cost * M / dt²`` using the home 7×7 mass table

Each helper below is one official constraint. ``solver.step`` assembles them
into the same 7×7 QP objective mink uses, then applies inequality-style
clips that official puts in DAQP.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from robot_runtime.openarm_ik.model import HingeJoint

# Official ARM_JOINT_VELOCITY_LIMITS_RAD_S. Opt-in in upstream; stored here
# so a planner can turn the cap on without hunting through mink.
ARM_JOINT_VELOCITY_LIMITS_RAD_S: tuple[float, ...] = (
    2.0,
    2.0,
    3.14,
    3.14,
    6.3,
    6.3,
    6.3,
)

# 7×7 arm inertia at home, extracted from OpenArm Cell via mj_fullM.
_MASS_RIGHT: tuple[tuple[float, ...], ...] = (
    (0.264794, -0.001271, 0.000693, 0.100744, 0.001316, -0.015211, -0.000969),
    (-0.001271, 0.164016, 0.096802, -0.001269, -0.000029, 0.000276, -0.011694),
    (0.000693, 0.096802, 0.261708, -0.000049, 0.000917, 0.000055, -0.015989),
    (0.100744, -0.001269, -0.000049, 0.261074, 0.001316, -0.016074, 0.000055),
    (0.001316, -0.000029, 0.000917, 0.001316, 0.010793, -0.000276, -0.000133),
    (-0.015211, 0.000276, 0.000055, -0.016074, -0.000276, 0.014467, -0.000055),
    (-0.000969, -0.011694, -0.015989, 0.000055, -0.000133, -0.000055, 0.009277),
)
_MASS_LEFT: tuple[tuple[float, ...], ...] = (
    (0.264794, -0.001360, 0.001284, -0.100744, 0.001316, -0.015211, -0.000969),
    (-0.001360, 0.164016, 0.096802, 0.001362, -0.000029, 0.000276, -0.011694),
    (0.001284, 0.096802, 0.261708, 0.000055, 0.000917, 0.000055, -0.015989),
    (-0.100744, 0.001362, 0.000055, 0.261074, -0.001316, 0.016074, -0.000055),
    (0.001316, -0.000029, 0.000917, -0.001316, 0.010793, -0.000276, -0.000133),
    (-0.015211, 0.000276, 0.000055, 0.016074, -0.000276, 0.014467, -0.000055),
    (-0.000969, -0.011694, -0.015989, -0.000055, -0.000133, -0.000055, 0.009277),
)


def arm_mass_matrix(side: str) -> tuple[tuple[float, ...], ...]:
    """Return the home-configuration 7×7 mass matrix for one arm."""

    if side == "right":
        return _MASS_RIGHT
    if side == "left":
        return _MASS_LEFT
    raise ValueError(f"arm side must be 'right' or 'left', got {side!r}")


@dataclass(frozen=True)
class IKConstraints:
    """Official IKParams fields that this kinematics-only solver can use."""

    position_cost: float = 12.0
    orientation_cost: float = 1.5
    lm_damping: float = 0.01
    damping: float = 0.1
    dt: float = 0.004
    max_iters: int = 5
    frame_position_error_limit: float = 0.02
    frame_orientation_error_limit: float = 0.25
    nullspace_return_rate: float = 1.6
    nullspace_max_speed: float = 1.0
    nullspace_ratio_low: float = 0.02
    nullspace_ratio_high: float = 0.05
    singularity_max_approach_rate: float = 0.25
    singularity_ratio_stop: float = 0.02
    singularity_ratio_slow: float = 0.08
    singularity_braking_exponent: float = 2.0
    singularity_gradient_epsilon: float = 1e-4
    jacobian_characteristic_length: float = 0.3
    joint_braking_distance: float = 0.2
    joint_limit_gain: float = 0.95
    nullspace_cost: float = 8.5
    # Official default is None (off). The tuple is the documented OpenArm cap.
    velocity_limits: tuple[float, ...] | None = None
    kinetic_energy_cost: float = 2e-5
    # Stationary planning matches official BoundedFrameTask: position
    # activation stays 0, orientation is always clipped.
    enable_position_bound: bool = False
    enable_orientation_bound: bool = True
    enable_task_weights: bool = True
    enable_nullspace: bool = True
    enable_singularity_limit: bool = True
    # Official braking is inside ArmJointLimit, which is off unless
    # velocity_limits is set.
    enable_joint_braking: bool = False

    def __post_init__(self) -> None:
        if self.max_iters < 1:
            raise ValueError("max_iters must be at least 1")
        if not 0.0 < self.joint_limit_gain <= 1.0:
            raise ValueError("joint_limit_gain must be in (0, 1]")
        for name in (
            "position_cost",
            "orientation_cost",
            "lm_damping",
            "damping",
            "dt",
            "frame_position_error_limit",
            "frame_orientation_error_limit",
            "nullspace_return_rate",
            "nullspace_max_speed",
            "nullspace_cost",
            "joint_braking_distance",
            "kinetic_energy_cost",
        ):
            if getattr(self, name) < 0.0:
                raise ValueError(f"{name} must not be negative")
        if self.singularity_gradient_epsilon <= 0.0:
            raise ValueError("singularity_gradient_epsilon must be positive")


OFFICIAL_CONSTRAINTS = IKConstraints()


def substep_dt(constraints: IKConstraints) -> float:
    """Official ``solve()`` splits ``dt`` across ``max_iters`` inner steps."""

    return constraints.dt / float(constraints.max_iters)


def bound_frame_error(
    error: Sequence[float],
    constraints: IKConstraints,
) -> tuple[float, ...]:
    """BoundedFrameTask: clip orientation always; position only if activated.

    Official ``limit_activation`` starts at 0 and stays 0 for a stationary
    target, so a planner should leave ``enable_position_bound`` false.
    """

    linear = tuple(float(value) for value in error[:3])
    angular = tuple(float(value) for value in error[3:])
    if constraints.enable_position_bound:
        position_limit = constraints.frame_position_error_limit / float(
            constraints.max_iters
        )
        linear = _clip_vector(linear, position_limit)
    if constraints.enable_orientation_bound:
        orientation_limit = constraints.frame_orientation_error_limit / float(
            constraints.max_iters
        )
        angular = _clip_vector(angular, orientation_limit)
    return linear + angular


def task_costs(constraints: IKConstraints) -> tuple[float, ...]:
    """Per-row FrameTask weights: three position, three orientation."""

    if not constraints.enable_task_weights:
        return (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    return (
        constraints.position_cost,
        constraints.position_cost,
        constraints.position_cost,
        constraints.orientation_cost,
        constraints.orientation_cost,
        constraints.orientation_cost,
    )


def weight_task(
    jacobian: Sequence[Sequence[float]],
    error: Sequence[float],
    constraints: IKConstraints,
) -> tuple[tuple[tuple[float, ...], ...], tuple[float, ...]]:
    """Scale Jacobian rows and the error by the FrameTask costs."""

    costs = task_costs(constraints)
    weighted_error = tuple(costs[row] * float(error[row]) for row in range(6))
    weighted_jacobian = tuple(
        tuple(costs[row] * float(jacobian[row][col]) for col in range(len(jacobian[0])))
        for row in range(6)
    )
    return weighted_jacobian, weighted_error


def singularity_ratio(
    jacobian: Sequence[Sequence[float]],
    constraints: IKConstraints = OFFICIAL_CONSTRAINTS,
) -> float:
    """σ_min / σ_max of the length-normalized 6×7 Jacobian.

    Linear rows are divided by ``jacobian_characteristic_length`` so metres
    and radians share one scale, matching official ``normalized_arm_jacobian``.
    """

    length = constraints.jacobian_characteristic_length
    columns = len(jacobian[0])
    scaled = []
    for row in range(6):
        scale = 1.0 / length if row < 3 and length > 0.0 else 1.0
        scaled.append(tuple(float(jacobian[row][col]) * scale for col in range(columns)))
    gram = tuple(
        tuple(
            sum(scaled[row][col] * scaled[other][col] for col in range(columns))
            for other in range(6)
        )
        for row in range(6)
    )
    eigenvalues = _symmetric_eigenvalues(gram)
    squares = tuple(max(0.0, value) for value in eigenvalues)
    sigma_max = math.sqrt(max(squares))
    if sigma_max < 1e-18:
        return 0.0
    return math.sqrt(min(squares)) / sigma_max


def nullspace_fade(ratio: float, constraints: IKConstraints) -> float:
    """Official nullspace gain fades out as the arm nears a singularity."""

    return _smoothstep(ratio, constraints.nullspace_ratio_low, constraints.nullspace_ratio_high)


def singularity_approach_rate(ratio: float, constraints: IKConstraints) -> float:
    """How fast ρ is allowed to fall this substep, in 1/s.

    Official: smoothstep((ρ-stop)/(slow-stop)) ** exponent, times max rate.
    """

    stop = constraints.singularity_ratio_stop
    slow = constraints.singularity_ratio_slow
    unit = _smoothstep(ratio, stop, slow)
    return constraints.singularity_max_approach_rate * (
        unit ** constraints.singularity_braking_exponent
    )


def scale_singularity_step(
    delta: Sequence[float],
    gradient: Sequence[float],
    ratio_now: float,
    constraints: IKConstraints,
) -> tuple[float, ...]:
    """Enforce -∇ρ · Δq ≤ r(ρ) dt by shrinking Δq along itself if needed."""

    if not constraints.enable_singularity_limit:
        return tuple(float(value) for value in delta)
    decrease = -sum(
        float(grad) * float(step) for grad, step in zip(gradient, delta)
    )
    allowed = singularity_approach_rate(ratio_now, constraints) * substep_dt(
        constraints
    )
    if decrease <= allowed:
        return tuple(float(value) for value in delta)
    if decrease <= 1e-18:
        return tuple(float(value) for value in delta)
    scale = allowed / decrease
    return tuple(float(value) * scale for value in delta)


def clip_joint_velocity(
    delta: Sequence[float],
    constraints: IKConstraints,
) -> tuple[float, ...]:
    """|Δq_i| ≤ v_i · (dt / max_iters) when velocity_limits is set."""

    limits = constraints.velocity_limits
    if limits is None:
        return tuple(float(value) for value in delta)
    dt = substep_dt(constraints)
    clipped = []
    for index, increment in enumerate(delta):
        cap = float(limits[index]) * dt if index < len(limits) else abs(float(increment))
        value = float(increment)
        clipped.append(max(-cap, min(cap, value)))
    return tuple(clipped)


def apply_joint_limits(
    angles: Sequence[float],
    delta: Sequence[float],
    hinges: Sequence[HingeJoint],
    constraints: IKConstraints,
) -> tuple[float, ...]:
    """ArmJointLimit: brake near the stop, then clip to [lower, upper]."""

    updated = []
    gain = constraints.joint_limit_gain
    for angle, increment, hinge in zip(angles, delta, hinges):
        step = float(increment)
        if constraints.enable_joint_braking and constraints.joint_braking_distance > 0.0:
            step = _brake_toward_stop(
                float(angle),
                step,
                hinge.lower,
                hinge.upper,
                constraints.joint_braking_distance,
            )
        room_upper = gain * (hinge.upper - float(angle))
        room_lower = gain * (hinge.lower - float(angle))
        step = min(room_upper, max(room_lower, step))
        updated.append(min(hinge.upper, max(hinge.lower, float(angle) + step)))
    return tuple(updated)


def _brake_toward_stop(
    angle: float,
    delta: float,
    lower: float,
    upper: float,
    distance: float,
) -> float:
    if delta > 0.0:
        room = upper - angle
        if room <= 0.0:
            return 0.0
        if room < distance:
            delta *= room / distance
    elif delta < 0.0:
        room = angle - lower
        if room <= 0.0:
            return 0.0
        if room < distance:
            delta *= room / distance
    return delta


def _clip_vector(vector: Sequence[float], limit: float) -> tuple[float, ...]:
    if limit <= 0.0:
        return tuple(float(value) for value in vector)
    norm = math.sqrt(sum(float(value) * float(value) for value in vector))
    if norm <= limit:
        return tuple(float(value) for value in vector)
    scale = limit / norm
    return tuple(float(value) * scale for value in vector)


def _smoothstep(value: float, low: float, high: float) -> float:
    if high <= low:
        return 1.0 if value >= high else 0.0
    if value <= low:
        return 0.0
    if value >= high:
        return 1.0
    t = (value - low) / (high - low)
    return t * t * (3.0 - 2.0 * t)


def _symmetric_eigenvalues(
    matrix: Sequence[Sequence[float]],
    sweeps: int = 40,
) -> tuple[float, ...]:
    """Jacobi eigenvalues of a small symmetric matrix (here 6×6 JJᵀ)."""

    size = len(matrix)
    work = [list(float(value) for value in row) for row in matrix]
    for _ in range(sweeps):
        pivot_row, pivot_col, largest = 0, 1, 0.0
        for row in range(size):
            for col in range(row + 1, size):
                candidate = abs(work[row][col])
                if candidate > largest:
                    largest = candidate
                    pivot_row, pivot_col = row, col
        if largest < 1e-14:
            break
        app = work[pivot_row][pivot_row]
        aqq = work[pivot_col][pivot_col]
        apq = work[pivot_row][pivot_col]
        tau = (aqq - app) / (2.0 * apq)
        t = math.copysign(1.0, tau) / (abs(tau) + math.sqrt(1.0 + tau * tau))
        cosine = 1.0 / math.sqrt(1.0 + t * t)
        sine = t * cosine
        work[pivot_row][pivot_row] = cosine * cosine * app - 2.0 * sine * cosine * apq + sine * sine * aqq
        work[pivot_col][pivot_col] = sine * sine * app + 2.0 * sine * cosine * apq + cosine * cosine * aqq
        work[pivot_row][pivot_col] = 0.0
        work[pivot_col][pivot_row] = 0.0
        for index in range(size):
            if index == pivot_row or index == pivot_col:
                continue
            aip = work[index][pivot_row]
            aiq = work[index][pivot_col]
            work[index][pivot_row] = work[pivot_row][index] = cosine * aip - sine * aiq
            work[index][pivot_col] = work[pivot_col][index] = sine * aip + cosine * aiq
    return tuple(work[index][index] for index in range(size))
