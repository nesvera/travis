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
    DEPENDS
    CATKIN_DEPENDS roscpp rospy std_msgs
    INCLUDE_DIRS src/cross_detector
    INCLUDE_DIRS src/lane_detector
    INCLUDE_DIRS src/zebra_detector
    INCLUDE_DIRS src/traffic_sign_detector
)

install(PROGRAMS
  src/cross_detector/cross_detector.py
  src/homography/homography.py
  src/lane_detector/lane_detector.py
  src/zebra_detector/zebra_detector.py
  src/traffic_sign_detector/traffic_sign_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
