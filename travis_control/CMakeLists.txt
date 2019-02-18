cmake_minimum_required(VERSION 2.8.3)
project(travis_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp rospy std_msgs
    INCLUDE_DIRS src/pid
)

install(PROGRAMS
  src/pid/pid.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
