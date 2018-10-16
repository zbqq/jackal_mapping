cmake_minimum_required(VERSION 2.8)
project(tm_mapping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  elevation_msgs
  interactive_markers
  pluginlib
  nav_core
)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS elevation_msgs
  DEPENDS PCL
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
  include
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_executable(tm_filter src/tm_filter.cpp)
target_link_libraries(tm_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(tm_map src/tm_map.cpp)
add_dependencies(tm_map ${catkin_EXPORTED_TARGETS} elevation_msgs_gencpp)
target_link_libraries(tm_map ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(tm_prm 	src/tm_prm.cpp src/library/kdtree.c)
add_dependencies(tm_prm ${catkin_EXPORTED_TARGETS} elevation_msgs_gencpp)
target_link_libraries(tm_prm ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_library(tm_planner_lib src/library/tm_planner.cpp)
target_link_libraries(tm_planner_lib ${catkin_LIBRARIES})