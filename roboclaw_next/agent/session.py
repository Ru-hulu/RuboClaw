"""Session state for one continuous Agent interaction."""

from __future__ import annotations

from dataclasses import dataclass, field
from uuid import uuid4

from roboclaw_next.agent.message import AgentMessage


@dataclass
class AgentSession:
    """保存一次连续交互中的消息、摘要和唯一标识。"""

    session_id: str = field(default_factory=lambda: uuid4().hex)
    messages: list[AgentMessage] = field(default_factory=list)
    summary: str | None = None
    summary_cursor: int = 0

    def append(self, message: AgentMessage) -> None:
        """将一条新消息追加到当前会话历史。"""

        self.messages.append(message)
