cmake_minimum_required(VERSION 3.8)
project(skai_hardware_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/skai_hardware.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

pluginlib_export_plugin_description_file(
  hardware_interface
  skai_hardware.xml
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(
  FILES skai_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()