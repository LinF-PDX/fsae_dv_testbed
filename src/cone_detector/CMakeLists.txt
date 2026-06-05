cmake_minimum_required(VERSION 3.8)
project(cone_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search)

add_executable(cone_detector_node src/cone_detector_node.cpp)

target_include_directories(cone_detector_node PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cone_detector_node ${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

ament_target_dependencies(cone_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

install(TARGETS cone_detector_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()