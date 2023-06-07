cmake_minimum_required(VERSION 2.8.3)
project(gps_trimble)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_publisher src/gps_node.cpp)

target_link_libraries(gps_publisher
  ${catkin_LIBRARIES}
)
