"""OpenArm reach tools."""

from __future__ import annotations

__all__ = ["register_openarm_reach_tools"]


def __getattr__(name: str):
    if name == "register_openarm_reach_tools":
        from .tool import register_openarm_reach_tools

        return register_openarm_reach_tools
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
