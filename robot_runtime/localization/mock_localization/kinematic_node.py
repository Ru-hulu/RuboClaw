"""ROS 2 node that provides mock localization from commanded velocity."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from roboclaw_interfaces.srv import GetMockLocalizationPose

from robot_runtime.control.differential_drive_mpc import Pose2D, propagate
from robot_runtime.debug.rviz_path import DebugPathPublisher


SIMULATION_DT = 0.1
WHEEL_BASE = 0.45
POSE_SERVICE_NAME = "/mock_localization/get_pose"


class MockLocalizationNode(Node):
    """Integrate velocity commands and publish a simulated robot posture."""

    def __init__(self) -> None:
        super().__init__("mock_localization")

        self.current_pose = Pose2D(x=40.0, y=40.0, yaw=0.0)
        self.linear_speed = 0.0
        self.angular_speed = 0.0

        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        self.posture_publisher = self.create_publisher(
            PoseStamped,
            "/robot_posture",
            10,
        )
        self.pose_service = self.create_service(
            GetMockLocalizationPose,
            POSE_SERVICE_NAME,
            self.handle_get_pose,
        )
        self.debug_path_publisher = DebugPathPublisher(
            self,
            "/robot_path",
            max_poses=2000,
        )
        self.simulation_timer = self.create_timer(
            SIMULATION_DT,
            self.simulation_tick,
        )
        self.get_logger().info(
            f"Mock localization started: {1.0 / SIMULATION_DT:.1f} Hz"
        )

    def cmd_vel_callback(self, message: Twist) -> None:
        """Cache the latest body velocity command."""

        self.linear_speed = message.linear.x
        self.angular_speed = message.angular.z

    def simulation_tick(self) -> None:
        """Advance one time step and publish the resulting posture."""

        half_wheel_difference = self.angular_speed * WHEEL_BASE / 2.0
        left_speed = self.linear_speed - half_wheel_difference
        right_speed = self.linear_speed + half_wheel_difference
        self.current_pose = propagate(
            self.current_pose,
            left_speed,
            right_speed,
            dt=SIMULATION_DT,
            wheel_base=WHEEL_BASE,
        )
        posture_message = self.make_posture_message()
        self.posture_publisher.publish(posture_message)
        self.debug_path_publisher.append_pose(posture_message)

    def handle_get_pose(
        self,
        request: GetMockLocalizationPose.Request,
        response: GetMockLocalizationPose.Response,
    ) -> GetMockLocalizationPose.Response:
        """Return the latest internal pose through a small ROS service."""

        del request
        response.success = True
        response.message = "Current mock localization pose."
        response.frame_id = "map"
        response.x = self.current_pose.x
        response.y = self.current_pose.y
        response.yaw = self.current_pose.yaw
        return response

    def make_posture_message(self) -> PoseStamped:
        """Represent the planar pose as a standard stamped ROS pose."""

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = self.current_pose.x
        message.pose.position.y = self.current_pose.y
        message.pose.orientation.z = math.sin(self.current_pose.yaw / 2.0)
        message.pose.orientation.w = math.cos(self.current_pose.yaw / 2.0)
        return message


def main() -> None:
    rclpy.init()
    node = MockLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Mock localization interrupted")
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
