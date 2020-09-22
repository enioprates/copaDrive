cmake_minimum_required(VERSION 2.8.3)
project(image_processing)

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization ros_its_msgs prius_msgs rospy)

add_message_files(
	FILES
	coords.msg
	drive_param.msg
	AckermannDriveStamped.msg
	error_control.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
