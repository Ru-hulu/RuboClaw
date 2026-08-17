# RoboClaw Next

This directory is a workspace for rethinking and rebuilding RoboClaw's future architecture.

It is intentionally separated from the current `roboclaw/` implementation so larger design experiments, architectural notes, and prototype modules can evolve without disturbing the existing codebase.

Current prototype areas:

- `llm/`: minimal OpenAI-compatible provider boundary for OpenAI and DeepSeek.
- `tools/`: Tool abstractions, MCP adapters, and standalone tool programs.
- `agent/`: AgentMessage, AgentSession, ContextBuilder, and AgentRuntime.
- `examples/`: small runnable examples for learning each boundary.

Main tool-call chain:

- `examples/RuboclawClient.py`: interactive LLM tool-call loop over MCP-backed tools.
- `examples/roboclaw_tool_server.py`: stdio MCP server that exposes RoboClaw tools.
- `tools/programs/`: standalone programs executed behind MCP Tool contracts.
