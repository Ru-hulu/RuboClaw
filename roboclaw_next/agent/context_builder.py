"""Build bounded model context from an Agent Session."""

from __future__ import annotations

import json
from dataclasses import dataclass

from roboclaw_next.agent.message import AgentMessage
from roboclaw_next.agent.session import AgentSession
from roboclaw_next.llm.openai_compatible import LLMProvider


@dataclass(frozen=True)
class _ConversationTurn:
    """一轮对话的消息区间。

    区间采用 [start, end)，start 是 user，end 是下一条 user 的位置。
    """

    start: int
    end: int
    complete: bool


class ContextBuilder:
    """保留近期完整轮次，并将更早的轮次压缩为滚动摘要。"""

    def __init__(self, provider: LLMProvider, keep_recent_turns: int = 2) -> None:
        if keep_recent_turns < 1:
            raise ValueError("keep_recent_turns must be at least 1.")
        self.provider = provider
        self.keep_recent_turns = keep_recent_turns

    async def build(self, session: AgentSession) -> list[AgentMessage]:
        """构造本次模型调用使用的消息，并在需要时更新历史摘要。"""

        system_end, turns = _split_turns(session.messages)

        completed_turns = [turn for turn in turns if turn.complete]
        compressible_turns = completed_turns[: -self.keep_recent_turns]
        # 这里拿到的是除了最近几轮的turn,因为最近几轮不压缩
        new_turns = [
            turn for turn in compressible_turns if turn.end > session.summary_cursor
        ] # 这里拿到的是还没有压缩的turn
        if new_turns:
            await self._update_summary(session, new_turns)

        context = list(session.messages[:system_end]) # 拿到messages数组中最前面的systems
        if session.summary:
            context.append(
                AgentMessage(
                    role="system",
                    content=f"以下是此前对话的摘要：\n{session.summary}",
                )
            )
        # 在system 后面进行追加summary的结果
        history_start = max(system_end, session.summary_cursor)
        context.extend(session.messages[history_start:]) # 这里是messages数组中被压缩信息后面的内容（近几次的对话）
        return context

    async def _update_summary(
        self,
        session: AgentSession,
        turns: list[_ConversationTurn],
    ) -> None:
        messages_to_summarize: list[AgentMessage] = []
        for turn in turns:
            messages_to_summarize.extend(session.messages[turn.start : turn.end])

        previous_summary = session.summary or "（暂无历史摘要）"
        serialized_messages = json.dumps(
            [message.to_provider_dict() for message in messages_to_summarize],
            ensure_ascii=False,
            indent=2,
        )
        response = await self.provider.chat_with_retry(
            [
                {
                    "role": "system",
                    "content": (
                        "你负责压缩 Agent 的历史对话。"
                        "请结合已有摘要和新增轮次，"
                        "只输出更新后的累计摘要。"
                        "保留用户目标、已确认决定、重要工具结果、"
                        "失败原因和未解决问题，不要添加推测。"
                    ),
                },
                {
                    "role": "user",
                    "content": (
                        f"已有摘要：\n{previous_summary}\n\n"
                        f"需要合并的新轮次：\n{serialized_messages}"
                    ),
                },
            ],
            tools=None,
            temperature=0,
        )
        if response.finish_reason == "error":
            raise RuntimeError(response.content or "Failed to summarize session history.")

        summary = (response.content or "").strip()
        if not summary:
            raise RuntimeError("Session summarization returned empty content.")

        session.summary = summary
        session.summary_cursor = turns[-1].end


def _split_turns(
    messages: list[AgentMessage],
) -> tuple[int, list[_ConversationTurn]]:
    system_end = 0
    while system_end < len(messages) and messages[system_end].role == "system":
        system_end += 1
    # 检查消息列表开头有多少条连续的 System Message。
    if any(message.role == "system" for message in messages[system_end:]):
        raise ValueError("System messages must appear only at the beginning of a Session.")

    if system_end < len(messages) and messages[system_end].role != "user":
        raise ValueError("A conversation turn must start with a user message.")

    # 每一轮对话都从一条 user message 开始，
    # 因此所有 user 的索引就是 Turn.start。
    user_message_indexes = [
        index
        for index in range(system_end, len(messages))
        if messages[index].role == "user"
    ]
    # 下一条 user 是当前 Turn 的右边界（不包含），同时也是下一轮的 start。
    # 最后一轮没有下一条 user，因此使用 len(messages) 作为右边界。
    turn_end_indexes = user_message_indexes[1:] + [len(messages)]
    turns = [
        _make_turn(messages, start, end) # 两个role = user之间的部分
        for start, end in zip(user_message_indexes, turn_end_indexes)
    ]

    if any(not turn.complete for turn in turns[:-1]):
        raise ValueError("A new user turn cannot start before the previous turn completes.")
    return system_end, turns


def _make_turn(
    messages: list[AgentMessage],
    start: int,
    end: int,
) -> _ConversationTurn:
    final_message = messages[end - 1]
    complete = final_message.role == "assistant" and not final_message.tool_calls
    # 最后一条的角色是Assistant，并且没有调用工具。
    return _ConversationTurn(start=start, end=end, complete=complete)
