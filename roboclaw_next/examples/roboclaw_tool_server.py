"""MCP server that exposes RoboClaw tools."""

from __future__ import annotations

import asyncio
import sys
from pathlib import Path

from mcp.server.fastmcp import FastMCP


mcp = FastMCP("RoboClaw Tool Server", json_response=True)


@mcp.tool()
async def add_integers(a: int, b: int) -> str:
    """Run the standalone integer addition program."""

    program_path = Path(__file__).with_name("integer_addition.py")
    process = await asyncio.create_subprocess_exec(
        sys.executable,
        str(program_path),
        "--a",
        str(a),
        "--b",
        str(b),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    stdout, stderr = await process.communicate()
    if process.returncode != 0:
        error_message = stderr.decode("utf-8").strip()
        raise RuntimeError(error_message or "Integer addition program failed.")
    return stdout.decode("utf-8").strip()


@mcp.tool()
def echo(text: str) -> str:
    """Return the input text through the MCP server."""

    return text


if __name__ == "__main__":
    mcp.run(transport="stdio")
