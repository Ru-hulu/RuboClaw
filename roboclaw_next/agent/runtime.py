"""Runtime that executes the RoboClaw Next Agent loop."""

from __future__ import annotations

from typing import Any

from roboclaw_next.agent.context_builder import ContextBuilder
from roboclaw_next.agent.message import AgentMessage
from roboclaw_next.agent.session import AgentSession
from roboclaw_next.llm.openai_compatible import LLMProvider
from roboclaw_next.tools import ToolExecutionContext, ToolRegistry, ToolResult


class AgentRuntime:
    """持有模型与工具，并驱动一次 Session 的 Agent 执行循环。"""

    def __init__(
        self,
        provider: LLMProvider,
        tool_registry: ToolRegistry,
        context_builder: ContextBuilder,
    ) -> None:
        self.provider = provider
        self.tool_registry = tool_registry
        self.context_builder = context_builder

    async def run(
        self,
        session: AgentSession,
        *,
        max_iterations: int = 4,
        trace: bool = False,
    ) -> str | None:
        """持续执行模型和工具调用，直到模型给出最终回答。"""

        # max_iterations 用于防止 Agent 进入无限工具调用循环。
        for iteration in range(1, max_iterations + 1):
            tool_definitions = self.tool_registry.definitions()
            if trace:
                print(
                    f"\n[agent] iteration {iteration}: "
                    "send messages and tool definitions to LLM"
                )
                print(f"[agent] available tools: {_tool_names(tool_definitions)}")

            # `await` 会暂停当前协程，直到模型调用完成并返回结果。
            # 如需让请求在后台执行，应显式使用 asyncio.create_task(...)
            context_messages = await self.context_builder.build(session)
            response = await self.provider.chat_with_retry(
                [message.to_provider_dict() for message in context_messages],
                tools=tool_definitions,
            )
            if trace:
                print(f"[llm] finish_reason: {response.finish_reason}")
                print(
                    "[llm] tool_calls: "
                    f"{[tool_call.name for tool_call in response.tool_calls]}"
                )
            if not response.has_tool_calls:
                session.append(AgentMessage(role="assistant", content=response.content))
                return response.content

            session.append(
                AgentMessage(
                    role="assistant",
                    content=response.content,
                    tool_calls=response.tool_calls,
                )
            )
            for tool_call in response.tool_calls:
                context = ToolExecutionContext(
                    tool_call_id=tool_call.id,
                    session_id=session.session_id,
                )
                if trace:
                    print(f"[tool] call {tool_call.name} with {tool_call.arguments}")
                try:
                    result = await self.tool_registry.invoke(
                        tool_call.name,
                        tool_call.arguments,
                        context,
                    )
                except Exception as exc:
                    # 即使工具调用抛出 Python 异常，也必须生成配对的 tool
                    # message，否则后续发送给 LLM 的消息序列是不完整的。
                    result = ToolResult(
                        content=f"Tool execution failed: {exc}",
                        is_error=True,
                    )
                if trace:
                    print(f"[tool] result: {result.as_text()}")
                session.append(
                    AgentMessage(
                        role="tool",
                        tool_call_id=tool_call.id,
                        name=tool_call.name,
                        content=result.as_text(),
                    )
                )

        return None


def _tool_names(tool_definitions: list[dict[str, Any]]) -> list[str]:
    names: list[str] = []
    for definition in tool_definitions:
        function = definition.get("function", {})
        name = function.get("name")
        if isinstance(name, str):
            names.append(name)
    return names
