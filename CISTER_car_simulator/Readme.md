# COPADRIVE Simulator
* Version: 	1.1
* Date:		08/04/2020
* Author: 	Enio Filho

## Requirements
* **OS**: Ubuntu 18.04
* ROS: Melodic
* Gazebo: Gazebo 9

## How to setup the project - COPADRIVE Simulator
````
- Download files from bitbucket (git clone https://enpvf@bitbucket.org/enpvf/inline.git)
- Delete ".cache" files from ~/CISTER_car_simulator/Build folder
- Open a new terminal inside ~/CISTER_car_simulator
- catkin_make
- run the simulator: 
	- source devel/setup.launch
	- roslaunch car_demo demo.launch (original copadrive version)
	- roslaunch car_demo demo_t.launch (INLINE copadrive version)
````
## Errors and solutions:
- You should follow the steps described in https://github.com/osrf/car_demo/pull/43/commits/fd7bcc74cbc502adb005b1b4bb8129c16c6cdf36
	OR
- Replace the following lines:
````
	car_demo/CMakeLists.txt
		(current)
		l 14:	find_package(gazebo 8 REQUIRED)	
		l 15:	find_package(ignition-msgs0 REQUIRED)
		(new)				
		l 14: 	find_package(gazebo 9 REQUIRED)
		l 15:	find_package(ignition-msgs1 REQUIRED)
	car_demo/plugins/gazebo_ros_block_laser.cpp
		(new)
		l 28:	#include <ignition/math/Pose3.hh>
		
		(current)
		l 85:	last_update_time_ = this->world_->GetSimTime();
		(new)
  		l 85: last_update_time_ = this->world_->SimTime();
		
		(current)
		l 408: 	math::Pose pose;
		l 409:  pose.pos.x = 0.5*sin(0.01*this->sim_time_.Double());
		l 410:  gzdbg << "plugin simTime [" << this->sim_time_.Double() << "] update pose [" << pose.pos.x << "]\n";
		(new)
		l 408:	ignition::math::Pose3d pose;
		l 409:  pose.Pos().X() = 0.5*sin(0.01*this->sim_time_.Double());
		l 410  gzdbg << "plugin simTime [" << this->sim_time_.Double() << "] update pose [" << pose.Pos().X() << "]\n";
````






