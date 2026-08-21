"""In-process OpenArm FK/IK used by the MCP tools.

This layer only converts the planner's tuples into JSON-friendly dictionaries.
It does not load MuJoCo or spawn a process.
"""

from __future__ import annotations

from collections.abc import Sequence

from robot_runtime.openarm_ik import ORIGIN_FRAME, ReachPlan, fk, plan_reach


def get_ee_pose(arm: str, joints: Sequence[float]) -> dict[str, object]:
    """Read the EE pose[7] in the arm_origin frame."""

    values = [float(value) for value in joints]
    return {
        "arm": arm,
        "frame": ORIGIN_FRAME,
        "pose": list(fk(arm, values)),
        "joints": values,
    }


def plan_to_xyz(
    arm: str,
    joints: Sequence[float],
    x: float,
    y: float,
    z: float,
) -> dict[str, object]:
    """Plan a reach to an arm_origin xyz target, keeping the current EE orientation."""

    plan = plan_reach(arm, joints, (x, y, z))
    return serialize_plan(plan)


def serialize_plan(plan: ReachPlan) -> dict[str, object]:
    """Convert a ReachPlan into a structured tool payload."""

    step_count = max(0, len(plan.points) - 1)
    error_mm = plan.final_error_m * 1000.0
    if plan.ok:
        message = (
            f"Reached the target in the {plan.frame} frame with "
            f"{error_mm:.1f} mm error after {step_count} IK steps."
        )
    else:
        message = (
            f"Stopped after {step_count} IK steps in the {plan.frame} frame; "
            f"final position error is {error_mm:.1f} mm."
        )
    return {
        "ok": plan.ok,
        "failure_reason": plan.failure_reason,
        "frame": plan.frame,
        "arm": plan.arm,
        "dt": plan.dt,
        "final_error_m": plan.final_error_m,
        "target_pose": list(plan.target_pose),
        "message": message,
        "points": [
            {
                "t": point.t,
                "q": list(point.q),
                "ee_pose": list(point.ee_pose),
            }
            for point in plan.points
        ],
    }
