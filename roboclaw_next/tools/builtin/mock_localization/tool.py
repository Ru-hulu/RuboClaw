"""MCP contracts for mock localization process management."""

from __future__ import annotations

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from pydantic import BaseModel, Field

from .program import (
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
