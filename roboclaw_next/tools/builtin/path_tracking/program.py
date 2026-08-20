"""Lifecycle management for the MPC path tracking ROS process."""

from __future__ import annotations

import asyncio
import json
import sys
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path

from ..hybrid_astar_planner.program import DEFAULT_PLAN_OUTPUT_PATH
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
    reference_path_file: str | None = None


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
        self._reference_path_file: str | None = None

    async def start(self, reference_path_file: str | None = None) -> TrackingStatus:
        """Start MPC after confirming that localization is running."""

        self._refresh_state()
        if self._state == TrackingState.RUNNING:
            if reference_path_file:
                self._message = (
                    "MPC path tracking process is already running; stop it "
                    "before changing reference_path_file."
                )
            return self._status()

        localization_status = await self._localization_manager.get_status()
        if localization_status.state != MockLocalizationState.RUNNING:
            self._state = TrackingState.FAILED
            self._reference_path_file = None
            self._message = (
                "Cannot start path tracking because mock localization is not running."
            )
            return self._status()

        repository_root = Path(__file__).resolve().parents[4]
        try:
            resolved_reference_path = self._resolve_reference_path_file(
                reference_path_file,
                repository_root,
            )
        except ValueError as exc:
            self._state = TrackingState.FAILED
            self._reference_path_file = None
            self._message = str(exc)
            return self._status()

        command = [
            sys.executable,
            "-m",
            "robot_runtime.control.differential_drive_mpc.ros_node",
        ]
        if resolved_reference_path is not None:
            command.extend(
                [
                    "--ros-args",
                    "-p",
                    f"reference_path_file:={resolved_reference_path}",
                ]
            )

        try:
            self._process = await asyncio.create_subprocess_exec(
                *command,
                cwd=repository_root,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
        except OSError as exc:
            self._state = TrackingState.FAILED
            self._reference_path_file = None
            self._message = str(exc)
            return self._status()

        self._state = TrackingState.RUNNING
        self._reference_path_file = (
            str(resolved_reference_path)
            if resolved_reference_path is not None
            else None
        )
        if self._reference_path_file is None:
            self._message = "MPC path tracking process started with built-in path."
        else:
            self._message = (
                "MPC path tracking process started with reference path file: "
                f"{self._reference_path_file}"
            )
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
            reference_path_file=self._reference_path_file,
        )

    def _resolve_reference_path_file(
        self,
        reference_path_file: str | None,
        repository_root: Path,
    ) -> Path | None:
        requested_path = (reference_path_file or "").strip()
        if requested_path:
            path = Path(requested_path).expanduser()
            if not path.is_absolute():
                path = repository_root / path
        elif DEFAULT_PLAN_OUTPUT_PATH.is_file():
            path = DEFAULT_PLAN_OUTPUT_PATH
        else:
            return None

        path = path.resolve()
        if not path.is_file():
            raise ValueError(f"Reference path file does not exist: {path}")
        self._validate_reference_path_file(path)
        return path

    def _validate_reference_path_file(self, path: Path) -> None:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(
                f"Reference path file is not readable JSON: {path}"
            ) from exc

        if not isinstance(payload, dict):
            raise ValueError(
                f"Reference path file must contain a JSON object: {path}"
            )
        if payload.get("success") is not True:
            message = payload.get("message") or "plan is not successful"
            raise ValueError(
                "Reference path file does not contain a successful Hybrid A* "
                f"plan: {message}"
            )
        waypoints = payload.get("waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError(
                f"Reference path file does not contain any waypoints: {path}"
            )
