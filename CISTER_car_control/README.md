# CISTER_car_control

## Directory Structure
*To be updated*

## Requirements
* **OS**: Ubuntu 16.04
* ROS Kinetic

## How to install requirements and setup the project
**Prerequisites**
* Install ROS
````
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
````

**Setup the project**
````
catkin_make
(if cmake complains about a different source directory:
delete build/CMakeCache.txt and build/catkin_make.cache, and rerun
catkin_make)
````

**Run the project**

* Start leader_control (car_demo already running):
````
source devel/setup.bash
rosrun car_control leader_keyboard_stable_speed [car_name]
(i.e rosrun car_control leader_keyboard_stable_speed car1)
````

*Start platooning/repeat for every car pair (leader_control already running):
````
*New Terminal*
source devel/setup.bash
rosrun car_control platooning [car_name_follower] [car_name_leader]
(i.e rosrun car_control platooning car2 car1)
````

* Start Image Processing (TODO: Change name):
````
TODO
````
||||
