cmake_minimum_required(VERSION 2.8.3)
project(laser_scan_to_map)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
)
catkin_package()
add_executable(laser_scan_to_map_node src/laser_scan_to_map_node.cpp)
target_link_libraries(laser_scan_to_map_node ${catkin_LIBRARIES})
