cmake_minimum_required(VERSION 2.8.3)
project(laser_scan_to_map)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  cv_bridge
  sensor_msgs
  rospy
  std_msgs
  image_transport
)
find_package(OpenCV 2)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_package()
add_executable(laser_scan_to_map_node src/laser_scan_to_map_node.cpp)
target_link_libraries(laser_scan_to_map_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
