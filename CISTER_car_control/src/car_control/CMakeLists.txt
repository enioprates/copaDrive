cmake_minimum_required(VERSION 2.8.3)
project(car_control)

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosbag rosconsole roscpp_serialization ros_its_msgs prius_msgs)
find_package(SDL2 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${SDL2_INCLUDE_DIRS}  imgui imgui/examples imgui/examples/libs/gl3w)
link_directories(${catkin_LIBRARY_DIRS} )

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

#add_library(${PROJECT_NAME}
  #imgui/imgui.cpp
  #imgui/imgui_demo.cpp
  #imgui/imgui_draw.cpp
  #imgui/imgui_widgets.cpp
  #imgui/examples/imgui_impl_sdl.cpp
  #imgui/examples/imgui_impl_opengl3.cpp
  #imgui/examples/libs/gl3w/GL/gl3w.c
#)

macro(rostutorial T)
  add_executable(${T} ${T}/${T}.cpp)

  add_dependencies(${T} ${catkin_EXPORTED_TARGETS})

  target_link_libraries(${T} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${SDL2_LIBRARIES} GL dl)

  install(TARGETS ${T}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

foreach(dir
    leader_keyboard_stable_speed
    leader_keyboard_record_rosbag
    rosbag_player
    platooning
    platooning_OBU
    platooning_OMNET
    scenario

    )
  rostutorial(${dir})
endforeach()

install(FILES
  launch/talker_listener.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
