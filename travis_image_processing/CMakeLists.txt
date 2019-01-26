cmake_minimum_required(VERSION 2.8.3)
project(travis_image_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES travis_image_processing
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/homography/homography.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
