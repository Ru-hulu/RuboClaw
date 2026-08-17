"""Let an LLM choose and call tools exposed by a local MCP server.

Run with DeepSeek:

    DEEPSEEK_API_KEY=... ROBOCLAW_LLM_PROVIDER=deepseek PYTHONPATH=. \
        uv run --no-project --with openai --with "mcp[cli]<2" \
        python roboclaw_next/examples/RuboclawClient.py

Run with OpenAI:

    OPENAI_API_KEY=... ROBOCLAW_LLM_PROVIDER=openai PYTHONPATH=. \
        uv run --no-project --with openai --with "mcp[cli]<2" \
        python roboclaw_next/examples/RuboclawClient.py
"""

from __future__ import annotations

import asyncio
import os
import sys
from pathlib import Path
from typing import cast

from roboclaw_next.agent import AgentMessage, AgentRuntime, AgentSession
from roboclaw_next.llm import create_llm_provider
from roboclaw_next.llm.types import ProviderName
from roboclaw_next.tools import (
    MCPClientRuntime,
    StdioMCPServerConfig,
    ToolRegistry,
    load_mcp_tools,
)


def resolve_provider_name() -> ProviderName:
    raw_provider = os.environ.get("ROBOCLAW_LLM_PROVIDER", "deepseek").lower()
    if raw_provider not in ("openai", "deepseek"):
        raise ValueError("ROBOCLAW_LLM_PROVIDER must be 'openai' or 'deepseek'.")
    return cast(ProviderName, raw_provider)


async def main() -> None:
    server_path = Path(__file__).with_name("example_mcp_server.py")
    config = StdioMCPServerConfig(
        name="example_mcp",
        command=sys.executable,
        args=[str(server_path)],
    )

    print(f"[mcp] connect server: {config.name}")
    print(f"[mcp] command: {config.command} {config.args}")
    async with MCPClientRuntime(config) as runtime:
        registry = ToolRegistry(await load_mcp_tools(runtime))

        print("[mcp] registered tools:")
        for name in registry.names:
            print(f"- {name}")

        provider = create_llm_provider(
            resolve_provider_name(),
            temperature=0,
            max_tokens=1024,
        )
        session = AgentSession(
            messages=[
                AgentMessage(
                    role="system",
                    content=(
                        "You are a tool-using assistant. For arithmetic tasks, "
                        "call the provided tool instead of calculating directly. "
                        "After tool execution, answer the user in Chinese."
                    ),
                ),
            ]
        )
        agent_runtime = AgentRuntime(provider, registry)

        print("\nEnter /exit to quit.")
        while True:
            try:
                user_input = (await asyncio.to_thread(input, "\nYou: ")).strip()
            except EOFError:
                break
            if user_input == "/exit":
                break
            if not user_input:
                continue

            session.append(AgentMessage(role="user", content=user_input))
            answer = await agent_runtime.run(session, trace=True)
            if answer is None:
                raise RuntimeError("LLM MCP tool loop did not produce a final answer.")

            print("\nAssistant:")
            print(answer)


if __name__ == "__main__":
    asyncio.run(main())
