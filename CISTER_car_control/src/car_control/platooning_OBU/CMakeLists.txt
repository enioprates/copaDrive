include_directories(${catkin_INCLUDE_DIRS})
add_executable(platooning_OBU platooning_OBU.cpp)
target_link_libraries(platooning_OBU ${catkin_LIBRARIES})
install(TARGETS platooning_OBU
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
