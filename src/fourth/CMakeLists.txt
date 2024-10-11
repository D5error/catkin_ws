cmake_minimum_required(VERSION 3.0.2)
project(fourth)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ar_location
#  CATKIN_DEPENDS ar_track_alvar_msgs geometry_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(fourth src/fourth.cpp)
target_link_libraries(fourth ${catkin_LIBRARIES})
