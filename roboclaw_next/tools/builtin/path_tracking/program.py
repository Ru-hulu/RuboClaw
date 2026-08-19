"""Lifecycle management for the MPC path tracking ROS process."""

from __future__ import annotations

import asyncio
import sys
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

from ..mock_localization.program import (
    MockLocalizationProcessManager,
    MockLocalizationState,
)


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
    pid: int | None = None
    return_code: int | None = None
    message: str | None = None


class PathTrackingProcessManager:
    """Start, inspect, and stop one MPC path tracking process."""

    def __init__(
        self,
        localization_manager: MockLocalizationProcessManager,
    ) -> None:
        self._localization_manager = localization_manager
        self._process: asyncio.subprocess.Process | None = None
        self._state = TrackingState.IDLE
        self._message: str | None = None

    async def start(self) -> TrackingStatus:
        """Start MPC after confirming that localization is running."""

        self._refresh_state()
        if self._state == TrackingState.RUNNING:
            return self._status()

        localization_status = await self._localization_manager.get_status()
        if localization_status.state != MockLocalizationState.RUNNING:
            self._state = TrackingState.FAILED
            self._message = (
                "Cannot start path tracking because mock localization is not running."
            )
            return self._status()

        repository_root = Path(__file__).resolve().parents[4]
        try:
            self._process = await asyncio.create_subprocess_exec(
                sys.executable,
                "-m",
                "robot_runtime.control.differential_drive_mpc.ros_node",
                cwd=repository_root,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
        except OSError as exc:
            self._state = TrackingState.FAILED
            self._message = str(exc)
            return self._status()

        self._state = TrackingState.RUNNING
        self._message = "MPC path tracking process started."
        await asyncio.sleep(0.2)
        self._refresh_state()
        return self._status()

    async def get_status(self) -> TrackingStatus:
        """Return the latest MPC process state."""

        self._refresh_state()
        return self._status()

    async def stop(self) -> TrackingStatus:
        """Stop the active MPC process; repeated calls are safe."""

        self._refresh_state()
        if self._process is None:
            self._state = TrackingState.STOPPED
            self._message = "MPC path tracking process is not running."
            return self._status()
        if self._process.returncode is not None:
            return self._status()

        self._process.terminate()
        try:
            await asyncio.wait_for(self._process.wait(), timeout=10.0)
        except TimeoutError:
            self._process.kill()
            await self._process.wait()

        self._state = TrackingState.STOPPED
        self._message = "MPC path tracking process stopped."
        return self._status()

    def _refresh_state(self) -> None:
        if (
            self._process is not None
            and self._process.returncode is not None
            and self._state == TrackingState.RUNNING
        ):
            self._state = (
                TrackingState.SUCCEEDED
                if self._process.returncode == 0
                else TrackingState.FAILED
            )
            self._message = (
                "MPC path tracking process completed."
                if self._process.returncode == 0
                else "MPC path tracking process exited unexpectedly."
            )

    def _status(self) -> TrackingStatus:
        return TrackingStatus(
            state=self._state,
            pid=self._process.pid if self._process is not None else None,
            return_code=(
                self._process.returncode if self._process is not None else None
            ),
            message=self._message,
        )
