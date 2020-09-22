include_directories(${catkin_INCLUDE_DIRS})
add_executable(scenario scenario.cpp)
target_link_libraries(scenario ${catkin_LIBRARIES})
install(TARGETS scenario
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
