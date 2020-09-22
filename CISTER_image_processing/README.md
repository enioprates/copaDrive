# CISTER_image_processing
* Version: 	1.0
* Date:		08/04/2020
* Author: 	Enio Filho

## Requirements
* **OS**: Ubuntu 18.04
* ROS: Melodic
* Gazebo: Gazebo 9

## InLINE Control algorithms
- the control algorithms are inside ~/CISTER_image_processing
- Delete ".cache" files from ~/CISTER_image_processing/Build folder
- Open a new terminal inside ~/CISTER_image_processing
- Compile
	- catkin_make		
	Error and solutions:
	- Install SDL2:	sudo apt-get install libsdl2-dev

* **INFO**: The control algorithm is based in several files, as long as the leader of the platoon follows the line in the road

## Python
- Be sure to make all files as executables 
````
- executable files: 	chmod +x <NAME_OF_THE_FILE>.py
- libraries:		sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose
````

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





