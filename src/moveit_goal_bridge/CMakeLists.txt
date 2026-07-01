cmake_minimum_required(VERSION 3.8)

project(moveit_goal_bridge)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)

add_executable(
  moveit_goal_bridge
  src/moveit_goal_bridge.cpp
)

ament_target_dependencies(
  moveit_goal_bridge
  rclcpp
  geometry_msgs
)

target_link_libraries(
  moveit_goal_bridge
  moveit_ros_planning_interface::moveit_move_group_interface
)

install(
  TARGETS moveit_goal_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()