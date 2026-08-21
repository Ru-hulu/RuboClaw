"""Forward kinematics for one OpenArm chain in the arm_origin frame.

No simulator: poses come from the serial product of translations and hinge
rotations stored in ``model.py``. The gripper slot in a length-8 command is
ignored; the EE site sits on the last arm body.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from robot_runtime.openarm_ik.model import arm


def fk(side: str, joints: Sequence[float]) -> tuple[float, ...]:
    """Map 7 arm joints (or q8 with a trailing gripper) to pose[7].

    pose = [px, py, pz, qw, qx, qy, qz] in the arm_origin frame, metres and
    unit quaternion.
    """

    transform, _, _ = _forward_chain(side, joints)
    return _pose_from_transform(transform)


def jacobian(side: str, joints: Sequence[float]) -> tuple[tuple[float, ...], ...]:
    """Geometric Jacobian of the EE in the arm_origin frame.

    Shape is 6×7: rows 0–2 are linear velocity (m/rad), rows 3–5 are angular
    velocity (1/rad). Column j is the twist produced by ``qdot_j = 1``.
    """

    transform, axes, pivots = _forward_chain(side, joints)
    ee = (transform[0][3], transform[1][3], transform[2][3])
    columns = []
    for axis, pivot in zip(axes, pivots):
        linear = _cross(axis, _sub(ee, pivot))
        columns.append(linear + axis)
    return tuple(tuple(column[row] for column in columns) for row in range(6))


def _joint_angles(side: str, joints: Sequence[float]) -> tuple[float, ...]:
    kinematics = arm(side)
    values = tuple(float(value) for value in joints)
    if len(values) == kinematics.q8_length:
        values = values[: kinematics.n_joints]
    if len(values) != kinematics.n_joints:
        raise ValueError(
            f"expected {kinematics.n_joints} arm joints or {kinematics.q8_length} "
            f"driver values, got {len(values)}"
        )
    return values


def _forward_chain(
    side: str,
    joints: Sequence[float],
) -> tuple[
    tuple[tuple[float, ...], ...],
    tuple[tuple[float, ...], ...],
    tuple[tuple[float, ...], ...],
]:
    kinematics = arm(side)
    values = _joint_angles(side, joints)
    transform = _translate(kinematics.base_from_origin)
    axes = []
    pivots = []
    for hinge, angle in zip(kinematics.hinges, values):
        rotation = (transform[0][:3], transform[1][:3], transform[2][:3])
        axes.append(_matvec3(rotation, hinge.axis))
        joint_origin = _matmul4(transform, _translate(hinge.origin))
        pivots.append((joint_origin[0][3], joint_origin[1][3], joint_origin[2][3]))
        transform = _matmul4(transform, _hinge_transform(hinge.origin, hinge.axis, angle))
    return transform, tuple(axes), tuple(pivots)


def _matvec3(
    matrix: tuple[tuple[float, ...], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def _cross(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _sub(
    left: tuple[float, float, float],
    right: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def _hinge_transform(
    origin: tuple[float, float, float],
    axis: tuple[float, float, float],
    angle: float,
) -> tuple[tuple[float, ...], ...]:
    return _homogeneous(_rotation_about_axis(axis, angle), origin)


def _translate(offset: tuple[float, float, float]) -> tuple[tuple[float, ...], ...]:
    return _homogeneous(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), offset)


def _homogeneous(
    rotation: tuple[tuple[float, ...], ...],
    origin: tuple[float, float, float],
) -> tuple[tuple[float, ...], ...]:
    return (
        (rotation[0][0], rotation[0][1], rotation[0][2], origin[0]),
        (rotation[1][0], rotation[1][1], rotation[1][2], origin[1]),
        (rotation[2][0], rotation[2][1], rotation[2][2], origin[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def _rotation_about_axis(
    axis: tuple[float, float, float],
    angle: float,
) -> tuple[tuple[float, ...], ...]:
    kx, ky, kz = axis
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus = 1.0 - cosine
    return (
        (
            cosine + kx * kx * one_minus,
            kx * ky * one_minus - kz * sine,
            kx * kz * one_minus + ky * sine,
        ),
        (
            ky * kx * one_minus + kz * sine,
            cosine + ky * ky * one_minus,
            ky * kz * one_minus - kx * sine,
        ),
        (
            kz * kx * one_minus - ky * sine,
            kz * ky * one_minus + kx * sine,
            cosine + kz * kz * one_minus,
        ),
    )


def _matmul4(
    left: tuple[tuple[float, ...], ...],
    right: tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(left[row][k] * right[k][col] for k in range(4)) for col in range(4))
        for row in range(4)
    )


def _pose_from_transform(transform: tuple[tuple[float, ...], ...]) -> tuple[float, ...]:
    position = (transform[0][3], transform[1][3], transform[2][3])
    rotation = (
        transform[0][:3],
        transform[1][:3],
        transform[2][:3],
    )
    return position + _quaternion_wxyz(rotation)


def _quaternion_wxyz(rotation: tuple[tuple[float, ...], ...]) -> tuple[float, ...]:
    m00, m01, m02 = rotation[0]
    m10, m11, m12 = rotation[1]
    m20, m21, m22 = rotation[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        wxyz = (
            0.25 * scale,
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
        )
    elif m00 >= m11 and m00 >= m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        wxyz = (
            (m21 - m12) / scale,
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
        )
    elif m11 >= m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        wxyz = (
            (m02 - m20) / scale,
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        wxyz = (
            (m10 - m01) / scale,
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
        )
    return wxyz
