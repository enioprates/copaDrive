cmake_minimum_required(VERSION 2.8.12)
project(ros_its_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
)

add_message_files(
  FILES
  CAM_simplified.msg
  platoon_dist.msg
  OMNET_CAM.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
