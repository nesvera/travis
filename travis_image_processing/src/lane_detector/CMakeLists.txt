add_executable(laneDetector lane_detector.cpp)
target_link_libraries(laneDetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  )
add_dependencies(laneDetector travis_image_processing_generate_messages_cpp)
