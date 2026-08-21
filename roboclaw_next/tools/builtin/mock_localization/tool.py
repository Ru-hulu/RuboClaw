"""MCP contracts for mock localization process management."""

from __future__ import annotations

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from pydantic import BaseModel, Field

from .program import (
    MockLocalizationPose,
    MockLocalizationProcessManager,
    MockLocalizationState,
    MockLocalizationStatus,
)


class MockLocalizationStatusResult(BaseModel):
    """Structured lifecycle status returned by localization tools."""

    state: MockLocalizationState = Field(description="Current process state.")
    pid: int | None = Field(default=None, description="Operating system process ID.")
    return_code: int | None = Field(
        default=None,
        description="Process return code after it exits.",
    )
    message: str | None = Field(
        default=None,
        description="Human-readable lifecycle information.",
    )


class MockLocalizationPoseResult(BaseModel):
    """Structured pose returned by the mock localization service."""

    success: bool = Field(
        description="Whether the current pose was read successfully.",
    )
    frame_id: str | None = Field(
        default=None,
        description="Coordinate frame for x, y, and yaw.",
    )
    x: float | None = Field(
        default=None,
        description="Current x coordinate in map-frame meters.",
    )
    y: float | None = Field(
        default=None,
        description="Current y coordinate in map-frame meters.",
    )
    yaw: float | None = Field(
        default=None,
        description="Current planar yaw in radians.",
    )
    message: str | None = Field(
        default=None,
        description="Human-readable pose query result.",
    )


def register_mock_localization_tools(
    mcp: FastMCP,
    process_manager: MockLocalizationProcessManager | None = None,
) -> None:
    """Register mock localization lifecycle Tools."""

    manager = process_manager or MockLocalizationProcessManager()

    @mcp.tool(
        name="start_mock_localization",
        title="Start Mock Localization",
        description="Start the ROS node that publishes simulated robot posture.",
        annotations=ToolAnnotations(
            readOnlyHint=False,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=False,
        ),
    )
    async def start_mock_localization() -> MockLocalizationStatusResult:
        """Start mock localization if it is not already running."""

        return _to_result(await manager.start())

    @mcp.tool(
        name="get_mock_localization_status",
        title="Get Mock Localization Status",
        description="Read the current mock localization process state.",
        annotations=ToolAnnotations(
            readOnlyHint=True,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=False,
        ),
    )
    async def get_mock_localization_status() -> MockLocalizationStatusResult:
        """Read the managed process status."""

        return _to_result(await manager.get_status())

    @mcp.tool(
        name="get_mock_localization",
        title="Get Mock Localization",
        description=(
            "Read the robot's current simulated localization pose. This calls "
            "the ROS 2 service /mock_localization/get_pose "
            "(roboclaw_interfaces/srv/GetMockLocalizationPose) and returns "
            "the current map-frame x, y, and yaw. Mock localization must "
            "already be running before this tool can return a pose."
        ),
        annotations=ToolAnnotations(
            readOnlyHint=True,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=False,
        ),
    )
    async def get_mock_localization() -> MockLocalizationPoseResult:
        """Read the current simulated robot pose."""

        return _to_pose_result(await manager.get_pose())

    @mcp.tool(
        name="stop_mock_localization",
        title="Stop Mock Localization",
        description="Stop the active mock localization ROS node.",
        annotations=ToolAnnotations(
            readOnlyHint=False,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=False,
        ),
    )
    async def stop_mock_localization() -> MockLocalizationStatusResult:
        """Stop the managed process."""

        return _to_result(await manager.stop())


def _to_result(status: MockLocalizationStatus) -> MockLocalizationStatusResult:
    return MockLocalizationStatusResult(
        state=status.state,
        pid=status.pid,
        return_code=status.return_code,
        message=status.message,
    )


def _to_pose_result(pose: MockLocalizationPose) -> MockLocalizationPoseResult:
    return MockLocalizationPoseResult(
        success=pose.success,
        frame_id=pose.frame_id,
        x=pose.x,
        y=pose.y,
        yaw=pose.yaw,
        message=pose.message,
    )
