"""Lightweight MPC controller for a differential-drive robot."""

from .controller import (
    DifferentialDriveMPC,
    MPCConfig,
    MPCResult,
    Pose2D,
    WheelCommand,
    propagate,
)

__all__ = [
    "DifferentialDriveMPC",
    "MPCConfig",
    "MPCResult",
    "Pose2D",
    "WheelCommand",
    "propagate",
]
