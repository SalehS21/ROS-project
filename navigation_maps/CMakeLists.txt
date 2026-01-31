cmake_minimum_required(VERSION 3.0.2)
project(navigation_maps)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  move_base
  amcl
  map_server
  tf
  tf2_ros
  costmap_2d
  nav_msgs
  geometry_msgs
  sensor_msgs
  actionlib
  actionlib_msgs
  rviz
  dynamic_reconfigure
  message_generation
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs move_base amcl map_server tf tf2_ros
  costmap_2d nav_msgs geometry_msgs sensor_msgs actionlib actionlib_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install map files
install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
catkin_install_python(PROGRAMS
  scripts/arduino_motor_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
