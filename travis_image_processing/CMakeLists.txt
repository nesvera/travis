cmake_minimum_required(VERSION 2.8.3)
project(travis_image_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

find_package( OpenCV REQUIRED )
find_package(cmake_modules REQUIRED)

catkin_python_setup()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp rospy std_msgs 
    INCLUDE_DIRS include
    INCLUDE_DIRS src/lane_detector
)


set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

add_subdirectory(src/lane_detector)

install(PROGRAMS
  src/homography/homography.py
  src/lane_detector/lane_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
