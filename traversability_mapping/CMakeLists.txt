cmake_minimum_required(VERSION 2.8)
project(traversability_mapping)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport
  # pcl library
  pcl_ros
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  elevation_msgs
  # visualization
  interactive_markers
  # planner plugin
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

# include directories
include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
)

# link directories
link_directories(
  include
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

###########
## Build ##
###########

# Range Image Filter
add_executable(traversability_filter src/traversability_filter.cpp)
target_link_libraries(traversability_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping
add_executable(traversability_map src/traversability_map.cpp)
add_dependencies(traversability_map ${catkin_EXPORTED_TARGETS} elevation_msgs_gencpp)
target_link_libraries(traversability_map ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# PRM Planner
add_executable(traversability_prm 	src/traversability_prm.cpp src/library/kdtree.c)
add_dependencies(traversability_prm ${catkin_EXPORTED_TARGETS} elevation_msgs_gencpp)
target_link_libraries(traversability_prm ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Path Planner
add_executable(traversability_path   src/traversability_path.cpp)
add_dependencies(traversability_path ${catkin_EXPORTED_TARGETS} elevation_msgs_gencpp)
target_link_libraries(traversability_path ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# move_base Planner Plugin
add_library(tm_planner_lib src/library/tm_planner.cpp)
target_link_libraries(tm_planner_lib ${catkin_LIBRARIES})