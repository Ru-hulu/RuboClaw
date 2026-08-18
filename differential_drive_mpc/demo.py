"""Run the controller in a small closed-loop kinematic simulation."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict

from .controller import DifferentialDriveMPC, MPCConfig, Pose2D, propagate


def build_reference(steps: int, dt: float) -> list[Pose2D]:
    """Create a smooth, time-parameterized path for the demo."""

    linear_speed = 0.55
    poses: list[Pose2D] = []
    for index in range(steps):
        x = linear_speed * dt * index
        y = 0.35 * math.sin(0.7 * x)
        slope = 0.35 * 0.7 * math.cos(0.7 * x)
        poses.append(Pose2D(x=x, y=y, yaw=math.atan(slope)))
    return poses


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=60)
    args = parser.parse_args()

    config = MPCConfig()
    controller = DifferentialDriveMPC(config)
    reference = build_reference(args.steps + config.horizon + 1, config.dt)
    pose = Pose2D(x=0.0, y=-0.20, yaw=0.0)
    last_result = None

    for step in range(args.steps):
        target_window = reference[step : step + config.horizon + 1]
        last_result = controller.solve(pose, target_window)
        command = last_result.command
        pose = propagate(
            pose,
            command.left_speed,
            command.right_speed,
            dt=config.dt,
            wheel_base=config.wheel_base,
        )

    assert last_result is not None
    target = reference[args.steps]
    position_error = math.hypot(pose.x - target.x, pose.y - target.y)
    print(
        json.dumps(
            {
                "final_pose": asdict(pose),
                "final_reference": asdict(target),
                "position_error": position_error,
                "last_command": asdict(last_result.command),
                "last_prediction_cost": last_result.cost,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
