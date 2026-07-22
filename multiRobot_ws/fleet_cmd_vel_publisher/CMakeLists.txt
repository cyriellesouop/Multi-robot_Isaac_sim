# ROS2's build tool, colcon, doesn't compile C++ itself; it delegates to CMake, which is the actual build system that knows how to compile .cpp files into executables/libraries and link them against dependencies.

cmake_minimum_required(VERSION 3.8)
project(fleet_cmd_vel_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)  
find_package(rclcpp REQUIRED)                                         # locate the rclcpp library on my system
find_package(geometry_msgs REQUIRED)                                  # locate the Twist message definitions

add_executable(publish_fleet_cmd_vel src/publish_fleet_cmd_vel.cpp)   # compile this file into an executable
ament_target_dependencies(publish_fleet_cmd_vel rclcpp geometry_msgs)

install(TARGETS  publish_fleet_cmd_vel
        DESTINATION lib/${PROJECT_NAME})                              # tell colcon where to put the built binary

ament_package() # stitches  the package into the same system every other ROS2 package (Nav2, rclcpp, geometry_msgs, etc.) uses
