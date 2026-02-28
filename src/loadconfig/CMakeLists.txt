cmake_minimum_required(VERSION 3.0.2)
project(loadconfig)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  interfaces
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS interfaces std_msgs
)

catkin_install_python(PROGRAMS
  scripts/loadconfig_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
