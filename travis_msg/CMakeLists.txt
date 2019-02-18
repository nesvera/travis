cmake_minimum_required(VERSION 2.8.3)
project(travis_msg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  LaneInfo.msg
  Point2D.msg
  SignInfo.msg
  SignsDetected.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package()