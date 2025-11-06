cmake_minimum_required(VERSION 3.8)
project(vacuum)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vac_interfaces REQUIRED)

# Executable for the publisher.
add_executable(talker src/robot.cpp)
ament_target_dependencies(talker rclcpp vac_interfaces)

# Executable for the subsciber.
add_executable(listener src/charger.cpp)
ament_target_dependencies(listener rclcpp vac_interfaces)

# Install section -> so 'ros2 run' can find the executable:
install(TARGETS
    talker
    listener
    DESTINATION lib/${PROJECT_NAME})

ament_package()
