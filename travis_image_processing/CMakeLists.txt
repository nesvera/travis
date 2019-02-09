cmake_minimum_required(VERSION 2.8.3)
project(travis_image_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
)

find_package(cmake_modules REQUIRED)

catkin_python_setup()

set(BUILD_FLAGS "-std=c++11 -Wuninitialized -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
include_directories(include)

add_subdirectory(src/lane_detector)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS roscpp rospy std_msgs 
    INCLUDE_DIRS include
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/homography/homography.py
  src/lane_detector/lane_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

nstall(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)