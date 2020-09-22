
//  ROSBAG Synchronized player
// BV (bffbv@isep.ipp.pt)

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <prius_msgs/Control.h>
#include <ros_its_msgs/ScenarioFail.h>
#include "ros/ros.h"
#include "ros/console.h"

#include <chrono>
#include <thread>


#include <boost/foreach.hpp>
#include <boost/format.hpp>
#define foreach BOOST_FOREACH
#include "rosgraph_msgs/Clock.h"
#include <fstream>
#include <cmath>
#include <string>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <stdio.h>

#include <sstream>
#include <stdlib.h>     /* srand, rand */



int main(int argc, char **argv)
{
rosbag::Bag bag;

std::stringstream node_name_ss;
node_name_ss << "scen";
ros::init(argc, argv, node_name_ss.str());
ros::NodeHandle nh;

double gazebotime,rosbagtime, gazebotime_ns, rosbagtime_ns;
int timediff_after_ns,timediff_ns;

std::stringstream car_name_ss;
car_name_ss << "/" << argv[1] << "/ScenarioFail";

ros::Publisher topic_pub = nh.advertise<ros_its_msgs::ScenarioFail>(car_name_ss.str(), 1000);
ros::Rate loop_rate(10);

std::cout << argv[2] << "fail injected" << std::endl;


while(ros::ok()){

ros::spinOnce();
ros_its_msgs::ScenarioFail msg_to_send;


msg_to_send.car_name = argv[1];
msg_to_send.FailType = argv[2];
topic_pub.publish(msg_to_send);
loop_rate.sleep();
}


}

