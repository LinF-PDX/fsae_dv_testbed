cmake_minimum_required(VERSION 3.8)
project(can_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(can_bridge_node src/can_bridge_node.cpp)
ament_target_dependencies(can_bridge_node
  rclcpp
  ackermann_msgs
  can_msgs
  std_msgs
)

install(TARGETS can_bridge_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
