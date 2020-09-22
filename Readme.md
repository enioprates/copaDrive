# INLINE README
* Version: 	1.0
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
* **ATTENTION**: Before running the control algorithms, PAUSE the simulation and reset the time!!!!
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
## Python 
````
- executable files: 	chmod +x <NAME_OF_THE_FILE>.py
- libraries:		sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
````

## InLINE Control algorithms
- the control algorithms are inside ~/CISTER_image_processing
- Delete ".cache" files from ~/CISTER_image_processing/Build folder
- Open a new terminal inside ~/CISTER_image_processing
- Compile
	- catkin_make		
	Error and solutions:
	- Install SDL2:	sudo apt-get install libsdl2-dev

* **INFO**: The control algorithm is based in several files, as long as the leader of the platoon follows the line in the road

## Starting the leader
````
(LINE DETECTION ALGORITHM)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing lane_lines_detection.py
````
````
(FOLLOW LINE ALGORITHM)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing simulation_connector.py
````

## Starting the Follower
````
(FOLLOWER)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing platooning.py <leader> <follower>
	example: rosrun image_processing platooning.py car1 car2
````
## Starting Follower's follower
````
(FOLLOWER)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing platooning.py <leader> <follower> control
	example: rosrun image_processing platooning.py car2 car3 control
````
## Recording Data
````
(FOLLOWER)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing listener.py car1 car2		//records the leader and fisrt follower data

- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.launch
- rosrun image_processing listener.py <follower>	//records the remain followers data
````

## Adjustable paramters inside platooning.py:
````
- Steering PID
- Distance PID 
- Minimum distance between vehicles
- Bearing control (ON OFF)
````





