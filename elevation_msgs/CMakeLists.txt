cmake_minimum_required(VERSION 2.8.3)
project(elevation_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  nav_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  #MapMetaData.msg
  #OccupancyGrid.msg
  OccupancyElevation.msg
)

#add_service_files(
#  DIRECTORY srv
#  FILES
#  empty.srv)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elevation_msgs
  CATKIN_DEPENDS 
  message_runtime 
  message_generation
  geometry_msgs 
  std_msgs
  nav_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)



