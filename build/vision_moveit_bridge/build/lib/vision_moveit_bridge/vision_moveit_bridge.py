#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped


class VisionMoveitBridge(Node):

    def __init__(self):

        super().__init__("vision_moveit_bridge")

        self.subscription = self.create_subscription(
            Point,
            "/vision_target",
            self.target_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "/vision_goal_pose",
            10
        )

        self.get_logger().info(
            "Vision MoveIt Bridge Started"
        )

    def target_callback(self, msg):

        pose = PoseStamped()

        pose.header.frame_id = "BASE"

        pose.pose.position.x = msg.x / 1000.0
        pose.pose.position.y = msg.y / 1000.0
        pose.pose.position.z = msg.z / 1000.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        self.pose_pub.publish(pose)

        self.get_logger().info(
            f"\nPublished Pose"
            f"\nX={pose.pose.position.x:.3f}"
            f"\nY={pose.pose.position.y:.3f}"
            f"\nZ={pose.pose.position.z:.3f}"
        )


def main():

    rclpy.init()

    node = VisionMoveitBridge()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()