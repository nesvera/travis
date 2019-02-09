add_executable(laneDetector lane_detector.cpp)
target_link_libraries(laneDetector ${catkin_LIBRARIES})
#add_dependencies(gpsHemisphereInterface autorally_msgs_gencpp)

install(TARGETS laneDetector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)