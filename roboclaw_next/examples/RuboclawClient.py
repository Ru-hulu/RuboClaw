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
from typing import cast

from roboclaw_next.agent import AgentMessage, AgentRuntime, AgentSession, ContextBuilder
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
    config = StdioMCPServerConfig(
        name="roboclaw_tools",
        command=sys.executable,
        args=["-m", "roboclaw_next.tools.mcp_server"],
        env=os.environ.copy(),
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
                        "You are a tool-using assistant. Use the provided tools "
                        "when appropriate, and answer the user in Chinese. "
                        "When the requested operation has succeeded, give a final "
                        "answer instead of repeating status checks."
                    ),
                ),
            ]
        )
        context_builder = ContextBuilder(provider, keep_recent_turns=2)
        agent_runtime = AgentRuntime(provider, registry, context_builder)

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
                print("\nAssistant:")
                print(
                    "工具调用已经达到本轮上限，但客户端会继续运行。"
                    "你可以继续输入下一条指令，或查询当前状态。"
                )
                continue

            print("\nAssistant:")
            print(answer)


if __name__ == "__main__":
    asyncio.run(main())
