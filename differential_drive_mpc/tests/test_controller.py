"""Tests for the standalone differential-drive MPC controller."""

from __future__ import annotations

import math
import unittest

from differential_drive_mpc import (
    DifferentialDriveMPC,
    MPCConfig,
    Pose2D,
    propagate,
)


class DifferentialDriveModelTests(unittest.TestCase):
    def test_equal_wheel_speeds_move_straight(self) -> None:
        pose = propagate(
            Pose2D(0.0, 0.0, 0.0),
            1.0,
            1.0,
            dt=0.2,
            wheel_base=0.5,
        )

        self.assertAlmostEqual(pose.x, 0.2)
        self.assertAlmostEqual(pose.y, 0.0)
        self.assertAlmostEqual(pose.yaw, 0.0)

    def test_opposite_wheel_speeds_rotate_in_place(self) -> None:
        pose = propagate(
            Pose2D(1.0, 2.0, 0.0),
            -0.5,
            0.5,
            dt=0.1,
            wheel_base=0.5,
        )

        self.assertAlmostEqual(pose.x, 1.0)
        self.assertAlmostEqual(pose.y, 2.0)
        self.assertAlmostEqual(pose.yaw, 0.2)


class DifferentialDriveMPCTests(unittest.TestCase):
    def test_command_obeys_speed_and_acceleration_limits(self) -> None:
        config = MPCConfig(
            dt=0.1,
            horizon=6,
            max_wheel_speed=0.7,
            max_wheel_acceleration=1.0,
            optimizer_iterations=4,
        )
        controller = DifferentialDriveMPC(config)
        reference = [Pose2D(float(index), 0.0, 0.0) for index in range(7)]

        result = controller.solve(Pose2D(0.0, 0.0, 0.0), reference)

        self.assertLessEqual(abs(result.command.left_speed), 0.1 + 1e-9)
        self.assertLessEqual(abs(result.command.right_speed), 0.1 + 1e-9)
        self.assertLessEqual(abs(result.command.left_speed), 0.7)
        self.assertLessEqual(abs(result.command.right_speed), 0.7)

    def test_closed_loop_reduces_lateral_path_error(self) -> None:
        config = MPCConfig(horizon=8, optimizer_iterations=10)
        controller = DifferentialDriveMPC(config)
        pose = Pose2D(0.0, -0.3, 0.0)
        initial_error = abs(pose.y)

        for step in range(25):
            reference = [
                Pose2D(0.04 * (step + index), 0.0, 0.0)
                for index in range(config.horizon + 1)
            ]
            result = controller.solve(pose, reference)
            pose = propagate(
                pose,
                result.command.left_speed,
                result.command.right_speed,
                dt=config.dt,
                wheel_base=config.wheel_base,
            )

        self.assertLess(abs(pose.y), initial_error)
        self.assertTrue(math.isfinite(result.cost))


if __name__ == "__main__":
    unittest.main()
