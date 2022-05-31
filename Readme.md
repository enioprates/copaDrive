# INLINE README
* Version: 	1.1
* Date:		31/05/2022
* Author: 	Enio Filho

## Requirements
* **OS**: Ubuntu 18.04
* **ROS**: Melodic -> http://wiki.ros.org/melodic/Installation/Ubuntu
	- sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	- sudo apt update
	- sudo apt install ros-melodic-desktop-full
	- echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
	- source ~/.bashrc
* **Gazebo**: Gazebo 9
	- update Gazebo:
		- https://classic.gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0
		- if you get this error: gazebo: symbol lookup error: /usr/lib/x86_64-linux-gnu/libgazebo_common.so.9: undefined symbol: _ZN8ignition10fuel_tools12ClientConfig12SetUserAgentERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEE
		- do: "sudo apt upgrade libignition-math2"      ->(update libignition)
	- VMWare troubles (GAZEBO):
		- [Err] [REST.cc:205] Error in REST request
			- Inside ~/.ignition/fuel/config.yaml replace "url: https://api.ignitionfuel.org" by
			- url: https://api.ignitionrobotics.org
			- Ignition Fuel's default configuration file is stored under $HOME/.ignition/fuel/config.yaml
			- (https://github.com/ros-industrial/universal_robot/issues/412)
		- VMware: vmw_ioctl_command error Invalid argument
			- (https://programmerah.com/about-vmware-vmw_-ioctl_-command-error-invalid-argument-solution-36180/)
			- Set the environment variable to 0 under the terminal:
			- export SVGA_VGPU10=0       (temporary)
			- echo "export SVGA_VGPU10=0" >> ~/.bashrc        (permanently)

### Errors and solutions
- when running the 'sudo apt update' and facing the error “the following signatures couldn’t be verified because the public key is not available”, check the link: https://chrisjean.com/fix-apt-get-update-the-following-signatures-couldnt-be-verified-because-the-public-key-is-not-available/
	- or run the following instruction: sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys <PUBLIC_KEY_NUMBER>


## How to setup the project - COPADRIVE Simulator
- Check that ROS and Gazebo are installed
	- gazebo
- Clone Files
	- Download files from bitbucket (git clone https://enpvf@bitbucket.org/enpvf/inline.git)
	- Delete Build and Devel folders
	- Open a new terminal inside ~/CISTER_car_simulator
		- catkin_make
	- run the simulator: 
		- source devel/setup.bash
		- roslaunch car_demo demo.launch (original copadrive version)
		- roslaunch car_demo demo_t.launch (INLINE copadrive version)

* **ATTENTION**: Before running the control algorithms, PAUSE the simulation and reset the time!

## Errors and solutions:
- You should follow the steps described in https://github.com/osrf/car_demo/pull/43/commits/fd7bcc74cbc502adb005b1b4bb8129c16c6cdf36
- Replace the following lines:
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
- check the rosdep install:
	- sudo rosdep init
	- rosdep update
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
	- Install scipy: pip install scipy

* **INFO**: The control algorithm is based in several files, as long as the leader of the platoon follows the line in the road

## Starting the leader
````
(LINE DETECTION ALGORITHM)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.bash
- rosrun image_processing lane_lines_detection.py
````
````
(FOLLOW LINE ALGORITHM)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.bash
- rosrun image_processing simulation_connector.py


## Starting the Follower
(FOLLOWER)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.bash
- rosrun image_processing platooning.py <leader> <follower>
	- example: rosrun image_processing platooning.py car1 car2

## Starting Follower's follower
- (FOLLOWER)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.bash
- rosrun image_processing platooning.py <leader> <follower> control
	example: rosrun image_processing platooning.py car2 car3 control

## Recording Data
(FOLLOWER)
- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.bash
- rosrun image_processing listener.py car1 car2		//records the leader and fisrt follower data

- Open a new terminal inside ~/CISTER_image_processing
- source devel/setup.bash
- rosrun image_processing listener.py <follower>	//records the remain followers data
````

## Adjustable parameters inside platooning.py:
````
- Steering PID
- Distance PID 
- Minimum distance between vehicles
- Bearing control (ON OFF)
````

## Road and Vehicles positions:
````
- demo_t.launch
	- OVAL ROAD: <arg name="world_name" value="$(find car_demo)/worlds/LineTrack_oval2.world"/>
	- Vehicles Initial Position: <include file="$(find car_demo)/launch/cars_t_oval.launch"/>
	
	- CURVE ROAD: <arg name="world_name" value="$(find car_demo)/worlds/LineTrack_curve_03.world"/>
	- Vehicles Initial Position: <include file="$(find car_demo)/launch/cars_t_curve.launch"/>
	
	- CURVE WITH OBSTACLES ROAD: <arg name="world_name" value="$(find car_demo)/worlds/LineTrack_curve_extreme2.world"/>
	- Vehicles Initial Position: <include file="$(find car_demo)/launch/cars_t_curve.launch"/>
- When one is chosed, the others should be commented
````

## Launch Files:
````
In order to run several files together, we create some launch files:
- Leader and Follower(s)
	- Open a new terminal inside ~/CISTER_image_processing
	- source devel/setup.launch
	- roslaunch image_processing vehicles.launch
- Recording data
	- Open a new terminal inside ~/CISTER_image_processing
	- source devel/setup.launch
	- roslaunch image_processing listener.launch
````

## Changing the number of VEHICLES
````
To change the number of vehicles in GAZEBO:
- In ~/inline/CISTER_car_simulator/src/car_demo/launch verify the <include file="$(find car_demo)/launch/<NAME_OF_FILE>2.launch"/>
- open the file 
- Comment/uncomment the desired vehicles

To change the number of vehicles in OMNET:
- TBD
````
