from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Node(
        #     package="mock_camera",
        #     executable="mock_camera",
        #     output="screen"
        # ),

        Node(
            package="red_detector",
            executable="red_detector_node",
            output="screen"
        ),

        Node(
            package="vision_gui",
            executable="vision_gui",
            output="screen"
        ),

        Node(
            package="vision_moveit_bridge",
            executable="vision_moveit_bridge",
            output="screen"
        ),

        Node(
            package="moveit_goal_bridge",
            executable="moveit_goal_bridge",
            output="screen"
        ),
    ])