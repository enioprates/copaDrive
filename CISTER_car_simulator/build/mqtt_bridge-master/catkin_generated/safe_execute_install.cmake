execute_process(COMMAND "/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_simulator/build/mqtt_bridge-master/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_simulator/build/mqtt_bridge-master/catkin_generated/python_distutils_install.sh) returned error code ")
endif()