include_directories(${catkin_INCLUDE_DIRS})
add_executable(platooning platooning.cpp)
target_link_libraries(platooning ${catkin_LIBRARIES})
install(TARGETS platooning
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
