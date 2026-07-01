from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # ---------------- PACKAGES ----------------
    moveit_pkg = get_package_share_directory('skai_moveit_config')

    # ---------------- MOVEIT ----------------
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'demo.launch.py')
        )
    )

    # ---------------- CYLINDER NODE ----------------
    # (THIS IS YOUR FIX — MUST RUN AFTER MOVEIT STARTS)
    cylinder_node = TimerAction(
        period=5.0,   # wait for MoveIt to fully boot
        actions=[
            Node(
                package='skai_bringup',
                executable='add_leaf',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        moveit_launch,
        cylinder_node
    ])