#!/usr/bin/env python3

import threading
import tkinter as tk

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException


class VisionGUI(Node):

    def __init__(self):

        super().__init__("vision_gui")

        #
        # TF Listener
        #

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.raw_x = 150.0
        self.raw_y = 150.0
        self.raw_z = 500.0

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0

        self.capture_x = -976.0
        self.capture_y = 110.0
        self.capture_z = 276.0

        #
        # Camera -> Robot mapping for simulation
        #
        # Camera:
        # X = left/right
        # Y = up/down
        # Z = depth
        #
        # Robot:
        # X = forward
        # Y = sideways
        # Z = vertical
        #

        self.robot_x = self.raw_z
        self.robot_y = self.raw_x
        self.robot_z = self.raw_y

        self.sub = self.create_subscription(
            PointStamped,
            "/red_object_xyz",
            self.camera_callback,
            10
        )

        self.pub = self.create_publisher(
            Point,
            "/vision_target",
            10
        )

        self.root = tk.Tk()
        self.root.title("Vision GUI")

        tk.Label(
            self.root,
            text="RAW CAMERA COORDINATES",
            font=("Arial", 12, "bold")
        ).pack()

        self.raw_label = tk.Label(
            self.root,
            text="X: 0\nY: 0\nZ: 0",
            font=("Arial", 11)
        )
        self.raw_label.pack(pady=10)

        tk.Label(
            self.root,
            text="ROBOT COORDINATES",
            font=("Arial", 12, "bold")
        ).pack()

        self.robot_label = tk.Label(
            self.root,
            text="X: 0\nY: 0\nZ: 0",
            font=("Arial", 11)
        )
        self.robot_label.pack(pady=10)

        tk.Label(
            self.root,
            text="CURRENT ARM POSE",
            font=("Arial", 12, "bold")
        ).pack()

        self.capture_label = tk.Label(
            self.root,
            text="X: 0\nY: 0\nZ: 0",
            font=("Arial", 11)
        )

        self.capture_label.pack(pady=10)

        self.capture_button = tk.Button(
            self.root,
            text="CAPTURE",
            command=self.capture_pose,
            width=20,
            height=2
        )

        self.capture_button.pack(pady=10)

        tk.Label(
            self.root,
            text="HARVEST TARGET",
            font=("Arial", 12, "bold")
        ).pack()

        self.harvest_label = tk.Label(
            self.root,
            text="X: 0\nY: 0\nZ: 0",
            font=("Arial", 11)
        )

        self.harvest_label.pack(pady=10)

        self.preview_button = tk.Button(
            self.root,
            text="PREVIEW",
            command=self.preview_target,
            width=20,
            height=2
        )

        self.preview_button.pack(pady=10)

        self.send_button = tk.Button(
            self.root,
            text="SEND TO MOVEIT",
            command=self.send_target,
            width=20,
            height=2
        )

        self.send_button.pack(pady=20)

        self.status_label = tk.Label(
            self.root,
            text="Waiting for target...",
            fg="blue"
        )

        self.status_label.pack()

        self.root.after(
            100,
            self.update_gui
        )

    def camera_callback(self, msg):

        #
        # Object lost
        #

        if (
            msg.point.x == 0.0 and
            msg.point.y == 0.0 and
            msg.point.z == 0.0
        ):

            self.status_label.config(
                text="OBJECT LOST"
            )

            return

        #
        # OAK-D coordinates
        #

        self.raw_x = msg.point.x
        self.raw_y = msg.point.y
        self.raw_z = msg.point.z
        #
        # Placeholder transform
        # Later:
        # camera frame -> robot frame
        #

        #
        # Camera -> Robot frame conversion
        #
        # Camera:
        # X = left/right
        # Y = up/down
        # Z = depth
        #
        # Robot:
        # X = forward
        # Y = sideways
        # Z = vertical
        #

        self.robot_x = self.raw_z
        self.robot_y = self.raw_x
        self.robot_z = self.raw_y

        #
        # Camera origin -> Gripper TCP
        #

        TCP_OFFSET_X = 0.0   # mm
        TCP_OFFSET_Y = 0.0
        TCP_OFFSET_Z = 0.0

        self.robot_x += TCP_OFFSET_X
        self.robot_y += TCP_OFFSET_Y
        self.robot_z += TCP_OFFSET_Z

        #
        # Harvest target calculation
        #
        # Camera:
        # X = left/right
        # Y = up/down
        # Z = depth
        #
        # Robot:
        # X = left/right
        # Y = forward
        # Z = up/down
        #

        self.harvest_x = (
            self.capture_x +
            self.robot_x
        )

        self.harvest_y = (
            self.capture_y +
            self.robot_y
        )

        self.harvest_z = (
            self.capture_z +
            self.robot_z
        )

    def capture_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                "BASE",
                "J_6",
                rclpy.time.Time()
            )

        except TransformException as ex:

            self.get_logger().warn(
                f"TF Error: {ex}"
            )

            return

        #
        # Convert meters -> mm
        #

        self.capture_x = (
            transform.transform.translation.x * 1000.0
        )

        self.capture_y = (
            transform.transform.translation.y * 1000.0
        )

        self.capture_z = (
            transform.transform.translation.z * 1000.0
        )

        self.status_label.config(
            text="Pose Captured"
        )

        self.get_logger().info(

            f"\n===== LIVE CAPTURE ====="

            f"\nX={self.capture_x:.1f}"

            f"\nY={self.capture_y:.1f}"

            f"\nZ={self.capture_z:.1f}"

        )

    def update_gui(self):

        self.raw_label.config(
            text=
            f"X: {self.raw_x:.1f} mm\n"
            f"Y: {self.raw_y:.1f} mm\n"
            f"Z: {self.raw_z:.1f} mm"
        )

        self.robot_label.config(
            text=
            f"X: {self.robot_x:.1f} mm\n"
            f"Y: {self.robot_y:.1f} mm\n"
            f"Z: {self.robot_z:.1f} mm"
        )

        self.capture_label.config(
            text=
            f"X: {self.capture_x:.1f} mm\n"
            f"Y: {self.capture_y:.1f} mm\n"
            f"Z: {self.capture_z:.1f} mm"
        )
        #
        # Continuously update harvest target
        #

        self.harvest_x = (
            self.capture_x +
            self.robot_x
        )

        self.harvest_y = (
            self.capture_y +
            self.robot_y
        )

        self.harvest_z = (
            self.capture_z +
            self.robot_z
        )

        self.harvest_label.config(
            text=
            f"X: {self.harvest_x:.1f} mm\n"
            f"Y: {self.harvest_y:.1f} mm\n"
            f"Z: {self.harvest_z:.1f} mm"
        )


        self.root.after(
            100,
            self.update_gui
        )

    def preview_target(self):

        self.get_logger().info(
            f"\n===== HARVEST PREVIEW ====="
            f"\nCapture:"
            f"\nX={self.capture_x:.1f}"
            f"\nY={self.capture_y:.1f}"
            f"\nZ={self.capture_z:.1f}"
            f"\n\nCamera:"
            f"\nX={self.raw_x:.1f}"
            f"\nY={self.raw_y:.1f}"
            f"\nZ={self.raw_z:.1f}"
            f"\n\nHarvest:"
            f"\nX={self.harvest_x:.1f}"
            f"\nY={self.harvest_y:.1f}"
            f"\nZ={self.harvest_z:.1f}"
        )


    def send_target(self):

        msg = Point()

        msg.x = self.harvest_x
        msg.y = self.harvest_y
        msg.z = self.harvest_z

        self.pub.publish(msg)

        self.status_label.config(
            text="Harvest Target Sent"
        )

        self.get_logger().info(
            f"\nSent Harvest Target\n"
            f"X={msg.x:.1f}\n"
            f"Y={msg.y:.1f}\n"
            f"Z={msg.z:.1f}"
        )


def ros_spin(node):

    rclpy.spin(node)


def main():

    rclpy.init()

    node = VisionGUI()

    spin_thread = threading.Thread(
        target=ros_spin,
        args=(node,),
        daemon=True
    )

    spin_thread.start()

    node.root.mainloop()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()