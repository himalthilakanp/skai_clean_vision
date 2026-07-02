#!/usr/bin/env python3

import rclpy
import time

from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from moveit_msgs.action import MoveGroup

from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PlanningScene,
    CollisionObject
)

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class IndustrialMotion(Node):

    def __init__(self):

        super().__init__("industrial_motion")

        # -------------------------------------------------
        # MOVEIT ACTION CLIENT
        # -------------------------------------------------
        self.client = ActionClient(
            self,
            MoveGroup,
            "move_action"
        )

        self.get_logger().info(
            "Waiting for MoveIt..."
        )

        self.client.wait_for_server()

        self.get_logger().info(
            "MoveIt ready ✔"
        )

        # -------------------------------------------------
        # GRIPPER ACTION CLIENT
        # -------------------------------------------------
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/GRIP_controller/follow_joint_trajectory"
        )

        self.get_logger().info(
            "Waiting for gripper..."
        )

        self.gripper_client.wait_for_server()

        self.get_logger().info(
            "Gripper ready ✔"
)

        # -------------------------------------------------
        # PLANNING SCENE PUBLISHER
        # -------------------------------------------------
        self.scene_pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            10
        )

        # add obstacle once
        self.add_small_cylinder()

    # -------------------------------------------------
    # ADD SMALL CYLINDER
    # -------------------------------------------------
    def add_small_cylinder(self):

        scene = PlanningScene()

        scene.is_diff = True

        # -------------------------------------------------
        # COLLISION OBJECT
        # -------------------------------------------------
        obj = CollisionObject()

        obj.id = "small_cylinder"

        obj.header.frame_id = "BASE"

        # -------------------------------------------------
        # CYLINDER SHAPE
        # -------------------------------------------------
        cylinder = SolidPrimitive()

        cylinder.type = SolidPrimitive.CYLINDER

        # [height, radius]
        cylinder.dimensions = [1.6, 0.015]

        # -------------------------------------------------
        # POSITION
        # -------------------------------------------------
        pose = Pose()

        pose.position.x = -0.45
        pose.position.y = -0.30

        # center of cylinder
        pose.position.z = 0.8

        pose.orientation.w = 1.0

        # -------------------------------------------------
        # ADD OBJECT
        # -------------------------------------------------
        obj.primitives.append(cylinder)

        obj.primitive_poses.append(pose)

        obj.operation = CollisionObject.ADD

        scene.world.collision_objects.append(obj)

        # -------------------------------------------------
        # PUBLISH
        # -------------------------------------------------
        self.scene_pub.publish(scene)

        self.get_logger().info(
            "Small cylinder added ✔"
        )

        time.sleep(2)

    # -------------------------------------------------
    # MOVE TO JOINT POSITION
    # -------------------------------------------------
    def move_to_position(self, joints, motion_name="PTP_MOVE"):

        goal = MoveGroup.Goal()

        # -------------------------------------------------
        # MOVE GROUP
        # -------------------------------------------------
        goal.request.group_name = "arm_group"

        # -------------------------------------------------
        # PILZ INDUSTRIAL PLANNER
        # -------------------------------------------------
        goal.request.pipeline_id = (
            "pilz_industrial_motion_planner"
        )

        goal.request.planner_id = "PTP"

        # -------------------------------------------------
        # SPEED
        # -------------------------------------------------
        goal.request.max_velocity_scaling_factor = 0.25

        goal.request.max_acceleration_scaling_factor = 0.25

        # -------------------------------------------------
        # JOINT NAMES
        # -------------------------------------------------
        joint_names = [
            
            "ROT_1",
            "PITCH_1",
            "PITCH_2",
            "PITCH_3",
            "ROT_2",
            "ROT_3"
        ]

        # -------------------------------------------------
        # CONSTRAINTS
        # -------------------------------------------------
        constraints = Constraints()

        for i in range(6):

            jc = JointConstraint()

            jc.joint_name = joint_names[i]

            jc.position = float(joints[i])

            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001

            jc.weight = 1.0

            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints.append(
            constraints
        )

        # -------------------------------------------------
        # SEND GOAL
        # -------------------------------------------------
        self.get_logger().info(
            f"Executing {motion_name}..."
        )

        send_future = self.client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        # -------------------------------------------------
        # CHECK ACCEPTED
        # -------------------------------------------------
        if not goal_handle.accepted:

            self.get_logger().error(
                f"{motion_name} rejected ❌"
            )

            return

        # -------------------------------------------------
        # WAIT RESULT
        # -------------------------------------------------
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = result_future.result()

        if result.result.error_code.val == 1:
            self.get_logger().info(
                f"{motion_name} completed ✔"
            )
        else:
            self.get_logger().error(
                f"{motion_name} failed. Error code: "
                f"{result.result.error_code.val}"
            )

        time.sleep(1)

    # -------------------------------------------------
    # GRIPPER MOTION
    # -------------------------------------------------
    def move_gripper(
        self,
        fgr1,
        fgr2,
        motion_name="GRIP_MOVE"
    ):

        goal = FollowJointTrajectory.Goal()

        goal.trajectory.joint_names = [
            "FGR_1",
            "FGR_2"
        ]

        point = JointTrajectoryPoint()

        point.positions = [
            float(fgr1),
            float(fgr2)
        ]

        point.time_from_start.sec = 2

        goal.trajectory.points.append(point)

        self.get_logger().info(
            f"Executing {motion_name}..."
        )

        send_future = (
            self.gripper_client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_future
        )

        goal_handle = send_future.result()

        if not goal_handle.accepted:

            self.get_logger().error(
                f"{motion_name} rejected ❌"
            )

            return

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        self.get_logger().info(
            f"{motion_name} completed ✔"
        )

        time.sleep(1)


    def open_gripper(self):

        self.move_gripper(
            0.0,
            0.0,
            "GRIP_OPEN"
        )


    def close_gripper(self):

        self.move_gripper(
            0.026,
            -0.026,
            "GRIP_CLOSE"
        )
    # -------------------------------------------------
    # RUN MOTIONS
    # -------------------------------------------------
    def run(self):

        # -------------------------------------------------
        # POSITIONS
        # -------------------------------------------------
        home = [ 0.0,0.0,0.0,0.0,0.0,0.0]

        p1 = [-0.382,-1.451, -2.520, -1.079,-0.493,0.00]
        p2 = [0.344,-0.186,-0.248,-0.055,-1.169,0.00]
        p21 = [-0.756,-0.238,-0.014,1.769,-0.779,0.00]
        #p1 = [0.122,0.368,-2.312,1.194,-0.090,0.00]
        p3 = [0.486,-0.353,-0.082,0.235,-1.180,0.000]
        p4 = [-0.517,-2.088,-1.824,0.227,-0.416,0.00]
        p5 = [0.179,0.534,-1.878,1.774,-0.085,0.00]
        p6 = [-0.364,-0.836,-0.394,1.831,-0.904,0.00]
        # -------------------------------------------------
        # EXECUTE SEQUENCE
        # -------------------------------------------------

        # clean startup home
        self.move_to_position(
            home,
            "START_HOME"
        )

        # move to P1
        self.move_to_position(
            p1,
            "P1"
        )
        
        self.open_gripper()

        # move to P2
        self.move_to_position(
            p2,
            "P2"
        )

        self.close_gripper()

        time.sleep(2)


         # move to P3
        self.move_to_position(
            p3,
            "P3"
        )

         # move to P4
        self.move_to_position(
            p4,
            "P4"
        )

         # move to P5
        self.move_to_position(
            p5,
            "P5"
        
        )
        # # move to P4
        # self.move_to_position(
        #     p6,
        #     "P6"
        # )

        # # IMPORTANT:
        # # intermediate safe pose
        # self.move_to_position(
        #     p1,
        #     "RETURN_P1"
        # )

        # final return home
        self.move_to_position(
            home,
            "RETURN_HOME"
        )

        self.get_logger().info(
            "Program completed ✔"
        )


# -------------------------------------------------
# MAIN
# -------------------------------------------------
def main(args=None):

    rclpy.init(args=args)

    node = IndustrialMotion()

    node.run()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()