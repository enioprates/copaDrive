include_directories(${catkin_INCLUDE_DIRS})
add_executable(leader_keyboard_stable_speed leader_keyboard_stable_speed.cpp)
target_link_libraries(leader_keyboard_stable_speed ${catkin_LIBRARIES})
install(TARGETS leader_keyboard_stable_speed
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
