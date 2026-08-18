"""MCP server that registers and exposes RoboClaw tools."""

from __future__ import annotations

from mcp.server.fastmcp import FastMCP

from .builtin.integer_addition import register_integer_addition
from .builtin.path_tracking import register_path_tracking_tools


mcp = FastMCP("RoboClaw Tool Server", json_response=True)

register_integer_addition(mcp)
register_path_tracking_tools(mcp)


if __name__ == "__main__":
    mcp.run(transport="stdio")
