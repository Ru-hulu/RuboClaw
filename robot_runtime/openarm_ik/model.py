"""OpenArm serial-chain constants, with no simulator dependency.

Numbers are taken once from the official OpenArm Cell MJCF. Runtime code
does not import MuJoCo. Every pose in this package is expressed in the
`arm_origin` site frame, which sits on the lifter; the lifter itself is
frozen and is not part of the seven-DoF arm.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


ORIGIN_FRAME = "arm_origin"
_HOME = (0.0, 0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class HingeJoint:
    """One revolute joint: parent-frame offset, unit axis, and limits in radians."""

    origin: tuple[float, float, float]
    axis: tuple[float, float, float]
    lower: float
    upper: float


@dataclass(frozen=True)
class ArmKinematics:
    """Fixed geometry of one 7-DoF OpenArm chain relative to arm_origin."""

    side: str
    origin_frame: str
    base_from_origin: tuple[float, float, float]
    hinges: tuple[HingeJoint, ...]
    home: tuple[float, ...]

    @property
    def n_joints(self) -> int:
        return len(self.hinges)

    @property
    def q8_length(self) -> int:
        return self.n_joints + 1


_RIGHT = ArmKinematics(
    side="right",
    origin_frame=ORIGIN_FRAME,
    base_from_origin=(0.0, -0.031, 0.0),
    hinges=(
        HingeJoint((0.0, -0.0625, 0.0), (0.0, -1.0, 0.0), -math.radians(80.0), math.radians(200.0)),
        HingeJoint((0.0, -0.06, 0.0), (-1.0, 0.0, 0.0), -math.radians(10.0), math.radians(190.0)),
        HingeJoint((0.0, 0.0, -0.06625), (0.0, 0.0, -1.0), -math.pi / 2.0, math.pi / 2.0),
        HingeJoint((0.0, 0.0, -0.15375), (0.0, -1.0, 0.0), 0.0, math.radians(140.0)),
        HingeJoint((0.0, 0.0, -0.0955), (0.0, 0.0, -1.0), -math.pi / 2.0, math.pi / 2.0),
        HingeJoint((0.0, 0.0, -0.1205), (0.0, 1.0, 0.0), -math.pi / 4.0, math.pi / 4.0),
        HingeJoint((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -math.pi / 2.0, math.pi / 2.0),
    ),
    home=_HOME,
)

_LEFT = ArmKinematics(
    side="left",
    origin_frame=ORIGIN_FRAME,
    base_from_origin=(0.0, 0.031, 0.0),
    hinges=(
        HingeJoint((0.0, 0.0625, 0.0), (0.0, 1.0, 0.0), -math.radians(200.0), math.radians(80.0)),
        HingeJoint((0.0, 0.06, 0.0), (-1.0, 0.0, 0.0), -math.radians(190.0), math.radians(10.0)),
        HingeJoint((0.0, 0.0, -0.06625), (0.0, 0.0, -1.0), -math.pi / 2.0, math.pi / 2.0),
        HingeJoint((0.0, 0.0, -0.15375), (0.0, -1.0, 0.0), 0.0, math.radians(140.0)),
        HingeJoint((0.0, 0.0, -0.0955), (0.0, 0.0, -1.0), -math.pi / 2.0, math.pi / 2.0),
        HingeJoint((0.0, 0.0, -0.1205), (0.0, -1.0, 0.0), -math.pi / 4.0, math.pi / 4.0),
        HingeJoint((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), -math.pi / 2.0, math.pi / 2.0),
    ),
    home=_HOME,
)

_ARMS = {"right": _RIGHT, "left": _LEFT}


def arm(side: str) -> ArmKinematics:
    """Return the kinematic table for ``right`` or ``left``."""

    try:
        return _ARMS[side]
    except KeyError:
        raise ValueError(f"arm side must be 'right' or 'left', got {side!r}") from None
