cmake_minimum_required(VERSION 3.8)
project(group_24_ex4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)

add_executable(turtlebot_server src/turtlebot_server.cpp)
ament_target_dependencies(turtlebot_server
    rclcpp 
    interfaces 
    sensor_msgs 
    geometry_msgs
    )

target_link_libraries(turtlebot_server
    opencv_highgui 
    )

add_executable(burrow_client src/burrow_client.cpp)
ament_target_dependencies(burrow_client
    rclcpp
    interfaces
    sensor_msgs
    )

install(TARGETS
    turtlebot_server
    burrow_client 
    DESTINATION lib/${PROJECT_NAME}
    )


install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    )
ament_package()
