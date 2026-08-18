"""Controller boundary for a future path tracking process."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class PathPose:
    """A planar pose on a planned path."""

    x: float
    y: float
    yaw: float


class PathTrackingController:
    """Runs the future high-frequency MPC control loop."""

    def run(self, path: Sequence[PathPose]) -> None:
        """Track a path until completion or cancellation."""

        raise NotImplementedError("Path tracking control is not implemented yet.")
