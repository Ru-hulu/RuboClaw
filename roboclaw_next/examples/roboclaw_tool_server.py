"""MCP server that exposes RoboClaw tools."""

from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path
from typing import Annotated

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from pydantic import BaseModel, Field


mcp = FastMCP("RoboClaw Tool Server", json_response=True)


# FastMCP 会根据这个返回模型生成 outputSchema，用它描述工具结果中
# 必须包含哪些字段，以及每个字段的数据类型和含义。
class AdditionResult(BaseModel):
    """Structured output returned by the integer addition tool."""

    a: int = Field(description="The first integer used in the calculation.")
    # 定义一个名为 a 的整数类型字段，并为它附加一段 Schema 描述
    b: int = Field(description="The second integer used in the calculation.")
    result: int = Field(description="The sum of a and b.")


@mcp.tool(
    name="add_integers",
    title="Add Integers",
    description="Add two integers by running the standalone addition program.",
    annotations=ToolAnnotations(
        readOnlyHint=True,  # 只读取或计算数据，不修改外部状态
        destructiveHint=False,  # 不执行删除、覆盖等破坏性操作
        idempotentHint=True,  # 使用相同参数重复调用不会产生额外副作用
        openWorldHint=False,  # 不访问互联网、远程服务或现实物理环境
    ),
)
async def add_integers(
    # FastMCP 会根据参数的类型注解和 Field 描述生成 inputSchema，
    # 用它告诉 MCP Client 和 LLM：调用工具时需要提供哪些参数。
    a: Annotated[int, Field(description="The first integer to add.")],
    b: Annotated[int, Field(description="The second integer to add.")],
) -> AdditionResult:
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

    output = json.loads(stdout.decode("utf-8"))
    return AdditionResult.model_validate(output)


if __name__ == "__main__":
    mcp.run(transport="stdio")
