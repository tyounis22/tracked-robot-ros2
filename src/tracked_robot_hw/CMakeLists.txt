cmake_minimum_required(VERSION 3.8)
project(tracked_robot_hw)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Executable
add_executable(motor_driver_node
  src/motor_driver_node.cpp
)

# Link dependencies
ament_target_dependencies(motor_driver_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# Install
install(TARGETS motor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

