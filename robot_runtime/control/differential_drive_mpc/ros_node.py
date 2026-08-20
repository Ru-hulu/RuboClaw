"""Minimal ROS 2 node that runs MPC from external posture feedback."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node

from robot_runtime.debug.rviz_path import DebugPathPublisher

from .controller import (
    DifferentialDriveMPC,
    MPCConfig,
    Pose2D,
    WheelCommand,
    normalize_angle,
)


PATH_STEPS = 60
REFERENCE_LINEAR_SPEED = 0.55


def _build_reference_path(steps: int, dt: float) -> list[Pose2D]:
    """Create the hard-coded path followed by this minimal node."""

    linear_speed = 0.55
    poses: list[Pose2D] = []
    for index in range(steps):
        x = linear_speed * dt * index
        y = 0.35 * math.sin(0.7 * x)
        slope = 0.35 * 0.7 * math.cos(0.7 * x)
        poses.append(Pose2D(x=x, y=y, yaw=math.atan(slope)))
    return poses


def _load_reference_path(path_file: str) -> list[Pose2D]:
    """Load a Hybrid A* plan JSON file as raw waypoints."""

    path = Path(path_file).expanduser()
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("reference_path_file must contain a JSON object")
    if payload.get("success") is not True:
        message = payload.get("message") or "plan is not successful"
        raise ValueError(
            f"reference_path_file does not contain a valid plan: {message}"
        )

    raw_waypoints = payload.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise ValueError("reference_path_file must contain at least one waypoint")

    waypoints: list[Pose2D] = []
    for index, raw_waypoint in enumerate(raw_waypoints):
        if not isinstance(raw_waypoint, dict):
            raise ValueError(f"waypoint {index} must be a JSON object")
        waypoint = _decode_waypoint(raw_waypoint, index)
        waypoints.append(waypoint)
    return waypoints


def _decode_waypoint(raw_waypoint: dict[str, Any], index: int) -> Pose2D:
    try:
        waypoint = Pose2D(
            x=float(raw_waypoint["x"]),
            y=float(raw_waypoint["y"]),
            yaw=float(raw_waypoint["yaw"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(
            f"waypoint {index} must contain numeric x, y, and yaw"
        ) from error

    values = (waypoint.x, waypoint.y, waypoint.yaw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"waypoint {index} contains a non-finite value")
    return waypoint


def _resample_reference_path(
    waypoints: list[Pose2D],
    *,
    step_distance: float,
) -> list[Pose2D]:
    """Resample sparse planner waypoints into MPC-sized reference steps."""

    if step_distance <= 0:
        raise ValueError("step_distance must be greater than zero")
    if len(waypoints) <= 1:
        return waypoints

    poses = [waypoints[0]]
    for start, end in zip(waypoints, waypoints[1:]):
        distance = math.hypot(end.x - start.x, end.y - start.y)
        segment_steps = max(1, math.ceil(distance / step_distance))
        yaw_delta = normalize_angle(end.yaw - start.yaw)
        for step in range(1, segment_steps + 1):
            ratio = step / segment_steps
            poses.append(
                Pose2D(
                    x=start.x + (end.x - start.x) * ratio,
                    y=start.y + (end.y - start.y) * ratio,
                    yaw=normalize_angle(start.yaw + yaw_delta * ratio),
                )
            )
    return poses


class DifferentialDriveMPCNode(Node):
    """Publish MPC commands using posture feedback from another ROS node."""

    def __init__(self) -> None:
        super().__init__("differential_drive_mpc")

        self.config = MPCConfig()
        self.controller = DifferentialDriveMPC(self.config)
        self.current_pose: Pose2D | None = None
        self.declare_parameter("reference_path_file", "")
        reference_path_file = (
            self.get_parameter("reference_path_file")
            .get_parameter_value()
            .string_value
            .strip()
        )
        if reference_path_file:
            raw_reference_path = _load_reference_path(reference_path_file)
            self.reference_path = _resample_reference_path(
                raw_reference_path,
                step_distance=REFERENCE_LINEAR_SPEED * self.config.dt,
            )
            self.reference_path_source = (
                f"{reference_path_file}: {len(raw_reference_path)} waypoints, "
                f"{len(self.reference_path)} MPC samples"
            )
        else:
            self.reference_path = _build_reference_path(
                PATH_STEPS + self.config.horizon + 1,
                self.config.dt,
            )
            self.reference_path_source = "built-in sine reference"
        self.path_steps = max(1, len(self.reference_path) - 1)
        self.step_index = 0
        self.finished = False

        self.posture_subscription = self.create_subscription(
            PoseStamped,
            "/robot_posture",
            self.posture_callback,
            10,
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.debug_reference_path_publisher = DebugPathPublisher(
            self,
            "/reference_path",
        )
        self.debug_reference_path_publisher.publish_planar_path(
            (pose.x, pose.y, pose.yaw)
            for pose in self.reference_path[: self.path_steps + 1]
        )
        self.control_timer = self.create_timer(self.config.dt, self.control_tick)
        self.get_logger().info(
            f"MPC started: {1.0 / self.config.dt:.1f} Hz, "
            f"{self.path_steps} steps, reference={self.reference_path_source}"
        )

    def posture_callback(self, message: PoseStamped) -> None:
        """Cache the latest posture received from the state node."""

        received_initial_posture = self.current_pose is None
        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        self.current_pose = Pose2D(
            x=message.pose.position.x,
            y=message.pose.position.y,
            yaw=yaw,
        )
        if received_initial_posture:
            self.get_logger().info("Received initial robot posture")

    def control_tick(self) -> None:
        """Solve one MPC step from the latest posture and publish it."""

        if self.current_pose is None:
            return

        if self.step_index >= self.path_steps:
            self.finish()
            return

        reference_window = self.reference_path[
            self.step_index : self.step_index + self.config.horizon + 1
        ]
        result = self.controller.solve(self.current_pose, reference_window)
        self.publish_command(result.command)
        self.step_index += 1

        if self.step_index == 1 or self.step_index % 10 == 0:
            self.get_logger().info(
                f"step={self.step_index:02d} "
                f"pose=({self.current_pose.x:.3f}, "
                f"{self.current_pose.y:.3f}, "
                f"{self.current_pose.yaw:.3f}) "
                f"wheels=({result.command.left_speed:.3f}, "
                f"{result.command.right_speed:.3f})"
            )

    def publish_command(self, command: WheelCommand) -> None:
        """Convert wheel speeds to the standard ROS body velocity command."""

        message = Twist()
        message.linear.x = command.linear_speed
        message.angular.z = command.angular_speed
        self.cmd_vel_publisher.publish(message)

    def publish_stop(self) -> None:
        """Publish a zero body velocity command."""

        self.cmd_vel_publisher.publish(Twist())

    def finish(self) -> None:
        """Stop the controller after the reference path is complete."""

        self.publish_stop()
        self.control_timer.cancel()
        self.finished = True
        target = self.reference_path[
            min(self.path_steps, len(self.reference_path) - 1)
        ]
        assert self.current_pose is not None
        position_error = math.hypot(
            self.current_pose.x - target.x,
            self.current_pose.y - target.y,
        )
        self.get_logger().info(
            f"MPC finished: final position error={position_error:.6f} m"
        )


def main() -> None:
    rclpy.init()
    node = DifferentialDriveMPCNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("MPC interrupted")
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
