"""Standard message value used by RoboClaw Next agents."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Literal

from roboclaw_next.llm.types import ToolCall


MessageRole = Literal["system", "user", "assistant", "tool"]


@dataclass
class AgentMessage:
    """表示 Session 中的一条文本消息或工具调用消息。"""

    role: MessageRole
    content: str | None = None
    tool_calls: list[ToolCall] = field(default_factory=list)
    tool_call_id: str | None = None
    name: str | None = None

    def to_provider_dict(self) -> dict[str, Any]:
        """转换成当前 OpenAI-compatible Provider 接受的消息字典。"""

        message: dict[str, Any] = {
            "role": self.role,
            "content": self.content,
        }
        if self.tool_calls:
            message["tool_calls"] = [
                tool_call.to_openai_tool_call() for tool_call in self.tool_calls
            ]
        if self.tool_call_id is not None:
            message["tool_call_id"] = self.tool_call_id
        if self.name is not None:
            message["name"] = self.name
        return message
