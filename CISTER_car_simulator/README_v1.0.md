# CISTER_car_simulator

## Directory Structure
*To be updated*

## Requirements
* **OS**: Ubuntu 16.04
* ROS Kinetic
* Gazebo 8

## How to install requirements and setup the project
**Prerequisites**
* Install Gazebo 8
````
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update 
sudo apt-get install -y gazebo8 ros-kinetic-gazebo8-ros-pkgs ros-kinetic-fake-localization ros-kinetic-joy
sudo apt-get clean
````

**Setup the project**
````
cd CISTER_car_simulator/
catkin_make

(if cmake complains about a different source directory:
delete build/CMakeCache.txt and build/catkin_make.cache, and rerun
catkin_make)
````

**Run the project**

* Start simulator:
````
source devel/setup.bash
roslaunch car_demo demo.launch
````

**ERRORS**
If error with xacro, apply code in catkin_workspace:
git clone https://github.com/ros/xacro/tree/kinetic-devel
