#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point


class MockCamera(Node):

    def __init__(self):

        super().__init__("mock_camera")

        self.pub = self.create_publisher(
            Point,
            "/mock_object_xyz",
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publish_target
        )

    def publish_target(self):

        msg = Point()

        msg.x = 100.0
        msg.y = 876.0
        msg.z = 0.0

        self.pub.publish(msg)

        self.get_logger().info(
            f"Published: "
            f"{msg.x}, "
            f"{msg.y}, "
            f"{msg.z}"
        )


def main():

    rclpy.init()

    node = MockCamera()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()