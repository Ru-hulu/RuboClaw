# RoboClaw Next 设计记录

## Session 与上下文管理

### 1. 基本定义

`AgentSession` 表示一段连续交互的上下文。它并不等同于一次机器人指令，也不只是类似 Codex 多窗口的界面概念。一次用户指令可以触发多轮 LLM 调用和工具执行，这些过程产生的信息都可以归入同一个 Session；如果后续用户输入依赖此前交流，也可以继续使用该 Session。

当前 RoboClaw 仍采用串行任务方式：前一条机器人指令未完成时，不处理下一条指令。Session 已经接管 Agent Loop 的消息记录，为后续的多轮交互和上下文管理提供明确的数据载体。

### 2. 职责边界

Session 与模型上下文不是同一个概念：

- `AgentSession` 负责保存“当前有哪些信息”。
- `ContextBuilder` 负责决定“本次模型调用使用哪些信息”。

LLM API 本身通常不保留上一次调用的内容。每次调用模型前，ContextBuilder 都需要从 Session 中读取相关信息，重新构造本次输入。

### 3. AgentSession 的最小结构

当前 `AgentSession` 只包含以下内容：

- `session_id`：唯一标识一段连续交互，后续可用于查找、恢复和持久化。
- `messages`：按产生顺序保存 `AgentMessage` 的数组。
- `summary`：对较早完整轮次形成的滚动摘要。
- `summary_cursor`：摘要已经覆盖到的消息位置，只能停在完整轮次末尾。
- `append(message)`：在新消息产生时，将其追加到 `messages`。

`messages` 的每个元素是一个 `AgentMessage`，表示一条消息，而不是一整个交互轮次。一轮工具调用可能包含多条消息：

```python
[
    AgentMessage(role="user", content="计算 19 + 23"),
    AgentMessage(role="assistant", tool_calls=[...]),
    AgentMessage(role="tool", content="42", tool_call_id="call-1"),
    AgentMessage(role="assistant", content="结果是 42"),
]
```

`AgentMessage` 当前只描述文本消息和工具调用消息。它在模型调用前转换成 OpenAI-compatible 字典，不包含持久化和多模态处理。

因此，`append()` 并不表示一轮交互结束。用户输入、Assistant 工具调用、工具执行结果和最终回答产生时，都会分别追加一条消息。

### 4. Summary 的处理原则

`AgentSession` 只保存摘要及其游标，不负责调用模型。`ContextBuilder` 按照用户轮次组织消息：一轮从 `user` 消息开始，到不包含 tool call 的最终 `assistant` 消息结束，中间的工具调用和工具结果不可拆分。

当前固定保留最近两个已完成轮次和正在执行的当前轮次。更早且尚未摘要的完整轮次会与已有摘要一起交给同一个 Provider，生成新的累计摘要。摘要成功后才更新 `summary` 和 `summary_cursor`，原始消息始终完整保留。

本次模型调用使用以下上下文：

```text
原始 System Message + 历史摘要 + 最近两个完整轮次 + 当前轮次
```

摘要是有损信息，机器人位置、物体状态和任务进度等关键状态不能只保存在摘要中，应结构化保存，并在必要时从仿真环境或真实硬件重新读取。

### 5. 当前实现范围

目前 `AgentSession`、`AgentMessage` 和 `ContextBuilder` 已接入 Agent Loop：模型响应、工具调用和工具结果都会写回 Session，Runtime 在每次模型调用前通过 ContextBuilder 构造上下文。当前按轮数管理上下文，尚未实现 token 预算、单轮超长处理、持久化和向量检索。

## Agent Runtime

`AgentRuntime` 持有 `provider` 和 `tool_registry`，并通过 `run(session)` 驱动模型调用、工具执行和消息回填。MCP connection 仍由外层 `MCPClientRuntime` 管理，Runtime 通过已经注册的 MCP Tool 间接使用该连接。

当前 Runtime 只负责单个 Session 的串行执行循环。`RuboclawClient.py` 会在同一条 MCP connection 中持续读取用户输入，并针对同一个 Session 重复调用 `run(session)`，从而保留多轮交互历史。上下文选择由 ContextBuilder 完成，任务状态和并发调度暂不属于这一版实现。
