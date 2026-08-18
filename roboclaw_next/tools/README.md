# Tools

This package defines the first minimal tool boundary for RoboClaw Next.

The core idea is:

```text
AgentTool
    -> ToolRegistry
    -> LLM tool schema / tool execution
```

## Files

- `base.py`: defines `AgentTool`, `ToolExecutionContext`, and `ToolResult`.
- `registry.py`: registers tools and invokes them by name.
- `mcp_runtime.py`: connects to a stdio MCP server with the official Python MCP SDK.
- `mcp_adapter.py`: converts MCP tools into the same `AgentTool` shape.
- `mcp_server.py`: creates the MCP server and registers concrete tools.
- `builtin/`: groups each concrete Tool's executable program and MCP contract.

Current built-in capabilities:

- `integer_addition/`: standalone addition program and its MCP Tool contract.
- `path_tracking/`: controller, process manager, and three MCP lifecycle Tool skeletons.

## MCP Tool-Call Chain

The main end-to-end example is:

```bash
DEEPSEEK_API_KEY=... ROBOCLAW_LLM_PROVIDER=deepseek PYTHONPATH=. \
    uv run --no-project --with openai --with "mcp[cli]<2" \
    python roboclaw_next/examples/RuboclawClient.py
```
