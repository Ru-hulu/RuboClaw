"""MCP contracts for path tracking process management."""

from __future__ import annotations

from typing import Annotated

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from pydantic import BaseModel, Field

from .controller import PathPose
from .process_manager import PathTrackingProcessManager, TrackingState, TrackingStatus


class PathPoseInput(BaseModel):
    """A planar pose supplied to the path tracking process."""

    x: float = Field(description="X position in meters.")
    y: float = Field(description="Y position in meters.")
    yaw: float = Field(description="Heading in radians.")


class TrackingStatusResult(BaseModel):
    """Structured lifecycle status returned by path tracking tools."""

    state: TrackingState = Field(description="Current path tracking process state.")
    message: str | None = Field(
        default=None,
        description="Optional process status or failure information.",
    )

 
def register_path_tracking_tools(
    mcp: FastMCP,
    process_manager: PathTrackingProcessManager | None = None,
) -> None:
    """Register the three path tracking lifecycle Tools."""

    manager = process_manager or PathTrackingProcessManager()

    @mcp.tool(
        name="start_path_tracking",
        title="Start Path Tracking",
        description="Start a controller process to follow a planned path.",
        annotations=ToolAnnotations(
            readOnlyHint=False,
            destructiveHint=False,
            idempotentHint=False,
            openWorldHint=True,
        ),
    )
    async def start_path_tracking(
        path: Annotated[
            list[PathPoseInput],
            Field(min_length=2, description="Ordered poses forming the planned path."),
        ],
    ) -> TrackingStatusResult:
        """Start the future path tracking process."""

        status = await manager.start(
            [PathPose(x=pose.x, y=pose.y, yaw=pose.yaw) for pose in path]
        )
        return _to_result(status)

    @mcp.tool(
        name="get_tracking_status",
        title="Get Tracking Status",
        description="Read the current path tracking process state.",
        annotations=ToolAnnotations(
            readOnlyHint=True,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=True,
        ),
    )
    async def get_tracking_status() -> TrackingStatusResult:
        """Read the future path tracking process status."""

        return _to_result(await manager.get_status())

    @mcp.tool(
        name="stop_path_tracking",
        title="Stop Path Tracking",
        description="Stop the active path tracking process.",
        annotations=ToolAnnotations(
            readOnlyHint=False,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=True,
        ),
    )
    async def stop_path_tracking() -> TrackingStatusResult:
        """Stop the future path tracking process."""

        return _to_result(await manager.stop())


def _to_result(status: TrackingStatus) -> TrackingStatusResult:
    return TrackingStatusResult(state=status.state, message=status.message)
