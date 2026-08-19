"""Lifecycle management for the mock localization ROS process."""

from __future__ import annotations

import asyncio
import sys
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path


class MockLocalizationState(StrEnum):
    """Lifecycle states exposed by the process manager."""

    IDLE = "idle"
    RUNNING = "running"
    STOPPED = "stopped"
    FAILED = "failed"


@dataclass(frozen=True)
class MockLocalizationStatus:
    """Current state of the managed ROS process."""

    state: MockLocalizationState
    pid: int | None = None
    return_code: int | None = None
    message: str | None = None


class MockLocalizationProcessManager:
    """Start, inspect, and stop one mock localization process."""

    def __init__(self) -> None:
        self._process: asyncio.subprocess.Process | None = None
        self._state = MockLocalizationState.IDLE
        self._message: str | None = None

    # 开始定位进程的位置
    async def start(self) -> MockLocalizationStatus:
        """Start the ROS node, or return its status if already running."""

        self._refresh_state()
        if self._state == MockLocalizationState.RUNNING:
            return self._status()

        repository_root = Path(__file__).resolve().parents[4]
        # 真正开始运行location模块的地方。
        try:
            self._process = await asyncio.create_subprocess_exec(
                sys.executable,
                "-m",
                "robot_runtime.localization.mock_localization.kinematic_node",
                cwd=repository_root,
                stdout=asyncio.subprocess.DEVNULL,
                stderr=asyncio.subprocess.DEVNULL,
            )
        except OSError as exc:
            self._state = MockLocalizationState.FAILED
            self._message = str(exc)
            return self._status()

        self._state = MockLocalizationState.RUNNING
        self._message = "Mock localization process started."
        await asyncio.sleep(0.2)
        self._refresh_state()
        return self._status()

    async def get_status(self) -> MockLocalizationStatus:
        """Return the latest process state."""

        self._refresh_state()
        return self._status()

    # 如何将定位进程停止
    async def stop(self) -> MockLocalizationStatus:
        """Stop the active process; repeated calls are safe."""

        self._refresh_state()
        if self._process is None or self._process.returncode is not None:
            self._state = MockLocalizationState.STOPPED
            self._message = "Mock localization process is not running."
            return self._status()

        self._process.terminate()
        try:
            await asyncio.wait_for(self._process.wait(), timeout=10.0)
        except TimeoutError:
            self._process.kill()
            await self._process.wait() # wait() 不是让子进程清理自己的资源，而是让父进程完成对子进程退出状态的处理。

        self._state = MockLocalizationState.STOPPED
        self._message = "Mock localization process stopped."
        return self._status()

    # 对进程状态进行刷新
    def _refresh_state(self) -> None:
        if (
            self._process is not None
            and self._process.returncode is not None
            and self._state == MockLocalizationState.RUNNING
        ):
            self._state = (
                MockLocalizationState.STOPPED
                if self._process.returncode == 0
                else MockLocalizationState.FAILED
            )
            self._message = (
                "Mock localization process exited."
                if self._process.returncode == 0
                else "Mock localization process exited unexpectedly."
            )

    def _status(self) -> MockLocalizationStatus:
        return MockLocalizationStatus(
            state=self._state,
            pid=self._process.pid if self._process is not None else None,
            return_code=(
                self._process.returncode if self._process is not None else None
            ),
            message=self._message,
        )
