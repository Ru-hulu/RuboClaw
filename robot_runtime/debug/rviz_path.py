"""Temporary RViz Path publishing helpers.

Keep visualization-only ROS code here so it can be removed without changing
the controller or localization algorithms.
"""

from __future__ import annotations

import math
from collections.abc import Iterable

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class DebugPathPublisher:
    """Publish either an accumulated trajectory or a fixed planar path."""

    def __init__(
        self,
        node: Node,
        topic: str,
        *,
        frame_id: str = "map",
        max_poses: int | None = None,
    ) -> None:
        self._node = node
        self._frame_id = frame_id
        self._max_poses = max_poses
        self._last_position: tuple[float, float] | None = None
        self._path = Path()
        self._path.header.frame_id = frame_id
        self._publisher = node.create_publisher(
            Path,
            topic,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def append_pose(self, pose: PoseStamped) -> None:
        """Append a moved robot pose and publish the accumulated trajectory."""

        position = (pose.pose.position.x, pose.pose.position.y)
        if self._last_position is not None:
            distance = math.hypot(
                position[0] - self._last_position[0],
                position[1] - self._last_position[1],
            )
            if distance < 1e-4:
                return

        self._last_position = position
        self._path.header.stamp = pose.header.stamp
        self._path.poses.append(pose)
        if self._max_poses is not None:
            del self._path.poses[: -self._max_poses]
        self._publisher.publish(self._path)

    def publish_planar_path(
        self,
        poses: Iterable[tuple[float, float, float]],
    ) -> None:
        """Replace and publish a fixed sequence of x, y, yaw poses."""

        stamp = self._node.get_clock().now().to_msg()
        self._path.header.stamp = stamp
        self._path.poses = [
            _make_pose_stamped(x, y, yaw, stamp, self._frame_id)
            for x, y, yaw in poses
        ]
        self._publisher.publish(self._path)


def _make_pose_stamped(
    x: float,
    y: float,
    yaw: float,
    stamp: Time,
    frame_id: str,
) -> PoseStamped:
    message = PoseStamped()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message
