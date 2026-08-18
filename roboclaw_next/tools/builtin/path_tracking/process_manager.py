"""Process lifecycle boundary for path tracking."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum
from typing import Sequence

from .controller import PathPose


class TrackingState(StrEnum):
    """Lifecycle states reported by the path tracking process."""

    IDLE = "idle"
    RUNNING = "running"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    STOPPED = "stopped"


@dataclass(frozen=True)
class TrackingStatus:
    """Internal status returned by the process manager."""

    state: TrackingState
    message: str | None = None


class PathTrackingProcessManager:
    """Starts, observes, and stops the future controller process."""

    async def start(self, path: Sequence[PathPose]) -> TrackingStatus:
        """Start one path tracking process."""

        raise NotImplementedError("Path tracking process startup is not implemented yet.")

    async def get_status(self) -> TrackingStatus:
        """Return the current path tracking process status."""

        raise NotImplementedError("Path tracking status is not implemented yet.")

    async def stop(self) -> TrackingStatus:
        """Stop the current path tracking process."""

        raise NotImplementedError("Path tracking process shutdown is not implemented yet.")
