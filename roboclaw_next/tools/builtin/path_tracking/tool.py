"""MCP contracts for path tracking process management."""

from __future__ import annotations

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from pydantic import BaseModel, Field

from .program import PathTrackingProcessManager, TrackingState, TrackingStatus


class TrackingStatusResult(BaseModel):
    """Structured lifecycle status returned by path tracking tools."""

    state: TrackingState = Field(description="Current path tracking process state.")
    pid: int | None = Field(default=None, description="Operating system process ID.")
    return_code: int | None = Field(
        default=None,
        description="Process return code after it exits.",
    )
    message: str | None = Field(
        default=None,
        description="Optional process status or failure information.",
    )


def register_path_tracking_tools(
    mcp: FastMCP,
    process_manager: PathTrackingProcessManager,
) -> None:
    """Register the three path tracking lifecycle Tools."""

    manager = process_manager

    @mcp.tool(
        name="start_path_tracking",
        title="Start Path Tracking",
        description=(
            "Start MPC using the currently configured reference path. "
            "Mock localization must already be running. Call "
            "get_mock_localization_status first; if it is not running, call "
            "start_mock_localization before starting path tracking."
        ),
        annotations=ToolAnnotations(
            readOnlyHint=False,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=False,
        ),
    )
    async def start_path_tracking() -> TrackingStatusResult:
        """Start MPC path tracking after checking localization."""

        return _to_result(await manager.start())

    @mcp.tool(
        name="get_tracking_status",
        title="Get Tracking Status",
        description="Read the current path tracking process state.",
        annotations=ToolAnnotations(
            readOnlyHint=True,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=False,
        ),
    )
    async def get_tracking_status() -> TrackingStatusResult:
        """Read the MPC path tracking process status."""

        return _to_result(await manager.get_status())

    @mcp.tool(
        name="stop_path_tracking",
        title="Stop Path Tracking",
        description="Stop the active path tracking process.",
        annotations=ToolAnnotations(
            readOnlyHint=False,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=False,
        ),
    )
    async def stop_path_tracking() -> TrackingStatusResult:
        """Stop the MPC path tracking process."""

        return _to_result(await manager.stop())


def _to_result(status: TrackingStatus) -> TrackingStatusResult:
    return TrackingStatusResult(
        state=status.state,
        pid=status.pid,
        return_code=status.return_code,
        message=status.message,
    )
