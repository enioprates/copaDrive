include_directories(${catkin_INCLUDE_DIRS})
add_executable(rosbag_player rosbag_player.cpp)
target_link_libraries(rosbag_player ${catkin_LIBRARIES})
install(TARGETS rosbag_player
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
