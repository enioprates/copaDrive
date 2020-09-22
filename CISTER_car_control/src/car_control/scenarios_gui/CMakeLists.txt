include_directories(${catkin_INCLUDE_DIRS})
add_executable(scenarios_gui scenarios_gui.cpp)
target_link_libraries(scenarios_gui ${catkin_LIBRARIES} imgui)
install(TARGETS scenarios_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
