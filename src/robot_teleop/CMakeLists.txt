cmake_minimum_required(VERSION 3.8)
project(robot_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(joy_to_ackermann_node src/joy_to_ackermann_node.cpp)
ament_target_dependencies(joy_to_ackermann_node
  rclcpp
  sensor_msgs
  ackermann_msgs
)

install(TARGETS joy_to_ackermann_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
