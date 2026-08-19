"""Minimal ROS 2 node that runs MPC from external posture feedback."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node

from robot_runtime.debug.rviz_path import DebugPathPublisher

from .controller import DifferentialDriveMPC, MPCConfig, Pose2D, WheelCommand


PATH_STEPS = 60


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


class DifferentialDriveMPCNode(Node):
    """Publish MPC commands using posture feedback from another ROS node."""

    def __init__(self) -> None:
        super().__init__("differential_drive_mpc")

        self.config = MPCConfig()
        self.controller = DifferentialDriveMPC(self.config)
        self.current_pose: Pose2D | None = None
        self.reference_path = _build_reference_path(
            PATH_STEPS + self.config.horizon + 1,
            self.config.dt,
        )
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
            for pose in self.reference_path[: PATH_STEPS + 1]
        )
        self.control_timer = self.create_timer(self.config.dt, self.control_tick)
        self.get_logger().info(
            f"MPC started: {1.0 / self.config.dt:.1f} Hz, {PATH_STEPS} steps"
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

        if self.step_index >= PATH_STEPS:
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
        """Stop the controller after the hard-coded path is complete."""

        self.publish_stop()
        self.control_timer.cancel()
        self.finished = True
        target = self.reference_path[PATH_STEPS]
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
