cmake_minimum_required(VERSION 3.8)
project(bag_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Added in our project:
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Executable for the subsciber.
add_executable(listener src/bag_listener.cpp)
ament_target_dependencies(listener rclcpp tf2 tf2_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
    listener
    DESTINATION lib/${PROJECT_NAME})


ament_package()
