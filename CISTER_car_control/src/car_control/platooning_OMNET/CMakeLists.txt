include_directories(${catkin_INCLUDE_DIRS})
add_executable(platooning_OMNET platooning_OMNET.cpp)
target_link_libraries(platooning_OMNET ${catkin_LIBRARIES})
install(TARGETS platooning_OMNET
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
