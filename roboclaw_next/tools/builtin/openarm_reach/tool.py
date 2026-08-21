"""MCP contracts for OpenArm forward and inverse kinematics."""

from __future__ import annotations

from typing import Annotated, Literal

from mcp.server.fastmcp import FastMCP
from mcp.types import ToolAnnotations
from pydantic import BaseModel, Field

from .program import get_ee_pose, plan_to_xyz


class EePoseResult(BaseModel):
    """Current end-effector pose in the arm_origin frame."""

    arm: Literal["right", "left"] = Field(description="Which OpenArm chain was queried.")
    frame: str = Field(description="Pose frame. Always arm_origin.")
    pose: list[float] = Field(
        description="EE pose [px, py, pz, qw, qx, qy, qz] in metres and unit quaternion.",
    )
    joints: list[float] = Field(
        description="Joint command used for FK: 7 arm joints, optionally plus gripper.",
    )


class TrajectoryPointResult(BaseModel):
    """One sample along the planned joint trajectory."""

    t: float = Field(description="Time from the start of the plan, in seconds.")
    q: list[float] = Field(description="Joint command at this sample.")
    ee_pose: list[float] = Field(description="EE pose[7] in the arm_origin frame.")


class ReachPlanResult(BaseModel):
    """Joint trajectory produced by one atomic OpenArm reach."""

    ok: bool = Field(description="True when the final position error is within tolerance.")
    failure_reason: str = Field(description="converged or max_steps.")
    frame: str = Field(description="Pose frame. Always arm_origin.")
    arm: Literal["right", "left"] = Field(description="Which OpenArm chain was planned.")
    dt: float = Field(description="Outer-loop sample period in seconds.")
    final_error_m: float = Field(description="Final Euclidean position error in metres.")
    target_pose: list[float] = Field(description="Assembled target pose[7] in arm_origin.")
    message: str = Field(description="Short summary for the model, including millimetre error.")
    points: list[TrajectoryPointResult] = Field(
        description="Trajectory samples, starting at the current configuration.",
    )


def register_openarm_reach_tools(mcp: FastMCP) -> None:
    """Register OpenArm pose and reach Tools."""

    @mcp.tool(
        name="get_openarm_ee_pose",
        title="Get OpenArm EE Pose",
        description=(
            "Read the OpenArm end-effector pose from the current joint command. "
            "Poses are in the arm_origin frame, in metres. joints is 7 hinge "
            "angles in radians, optionally followed by a gripper value that FK ignores."
        ),
        annotations=ToolAnnotations(
            readOnlyHint=True,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=False,
        ),
    )
    async def get_openarm_ee_pose(
        arm: Annotated[
            Literal["right", "left"],
            Field(description="Which arm to read."),
        ],
        joints: Annotated[
            list[float],
            Field(description="Current 7 arm joints, or q8 with a trailing gripper."),
        ],
    ) -> EePoseResult:
        """Return the current EE pose in the arm_origin frame."""

        return EePoseResult.model_validate(get_ee_pose(arm, joints))

    @mcp.tool(
        name="plan_openarm_reach",
        title="Plan OpenArm Reach",
        description=(
            "Plan a joint trajectory that moves one OpenArm chain to an xyz "
            "target in the arm_origin frame. Orientation is kept from the current "
            "end-effector pose. This is an IK calculation only; it does not command "
            "motors. Provide the current joints, then x, y, z in metres."
        ),
        annotations=ToolAnnotations(
            readOnlyHint=True,
            destructiveHint=False,
            idempotentHint=True,
            openWorldHint=False,
        ),
    )
    async def plan_openarm_reach(
        arm: Annotated[
            Literal["right", "left"],
            Field(description="Which arm to plan."),
        ],
        joints: Annotated[
            list[float],
            Field(description="Current 7 arm joints, or q8 with a trailing gripper."),
        ],
        x: Annotated[float, Field(description="Target x in the arm_origin frame, metres.")],
        y: Annotated[float, Field(description="Target y in the arm_origin frame, metres.")],
        z: Annotated[float, Field(description="Target z in the arm_origin frame, metres.")],
    ) -> ReachPlanResult:
        """Plan a reach and return the joint trajectory."""

        return ReachPlanResult.model_validate(plan_to_xyz(arm, joints, x, y, z))
