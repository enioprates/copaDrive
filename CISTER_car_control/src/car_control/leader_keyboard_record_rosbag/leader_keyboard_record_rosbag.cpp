// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros_its_msgs/CAM_simplified.h>
#include <prius_msgs/Control.h>
#include <sensor_msgs/LaserScan.h>
#include "ros/console.h"
#include <sstream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <cmath>
#include <string>
#include <iostream>
#include <cstdio>

#include <ctime>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <fstream>

#define ARGC_MIN_REQUIRED 2 // minimum required number of parameters

//Global vars
using namespace std;
std::string car_name;
std::string car_name_leader;
uint32_t car_id;


#define GEAR_NO_COMMAND 0 
#define GEAR_NEUTRAL    1 
#define GEAR_FORWARD    2 
#define GEAR_REVERSE    3


ros::Publisher pub_control_gazebo;

prius_msgs::Control control_pub_msg;
ros_its_msgs::CAM_simplified car_info_global;

double set_speed_global = 0;

rosbag::Bag bag;


double getTime(){
       // ROS_INFO("Time:%f", ros::Time::now().toSec());
        return ros::Time::now().toSec();
}

int help() {
  ROS_WARN("Missing [car_name]!");
  ROS_WARN("Usage: rosrun car_demo leader_control [car_name]");
  return 1;
}

double normalize(double x_max, double x_min, double value){
  return (value - x_min) / (x_max - x_min);
}

void car_info_Callback(const ros_its_msgs::CAM_simplified::ConstPtr& msg){
  //inputs (ros_its_msgs::CAM_simplified)
  //msg->latitude (float32)
  //msg->longitude (float64)
  //msg->altitude_altitudeValue (float32)
  //msg->heading_headingValue (float32)
  //msg->speed_speedValue (float32)
  //msg->driveDirection (int8)
  //msg->steeringWheelAngle_steeringWheelAngleValue (float32)
  //msg->gasPedalPercent_Value (float32)
  //msg->brakePedalPercent_Value (float32)
  car_info_global.speed_speedValue = msg->speed_speedValue;
  //getTime();
  
  //ROS_INFO("carINFO");
        
  return;
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)
  //fflush(stdin);
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


void keyboard_thread(int publish_rate)
{
  ros::Rate loop_rate(publish_rate);
  while (ros::ok())
  {
    int c = getch();
    if (c == 'a'){
        control_pub_msg.steer +=0.05;
        control_pub_msg.steer = std::min(control_pub_msg.steer, 1.0);
        control_pub_msg.steer = std::max(control_pub_msg.steer, -1.0);
        ROS_INFO("SET steering = %lf", control_pub_msg.steer);
    }
    else if (c == 'd'){
      control_pub_msg.steer -=0.05;
      control_pub_msg.steer = std::min(control_pub_msg.steer, 1.0);
      control_pub_msg.steer = std::max(control_pub_msg.steer, -1.0);
       ROS_INFO("SET steering = %lf", control_pub_msg.steer);
    } else if (c == 'w'){
        set_speed_global +=5; //km/h
        set_speed_global = std::min(set_speed_global, 100.0);
        set_speed_global = std::max(set_speed_global, 0.0);
        ROS_INFO("SET Speed = %lf", set_speed_global);
    } else if (c == 's'){
        set_speed_global -=5;
        set_speed_global = std::min(set_speed_global, 100.0);
        set_speed_global = std::max(set_speed_global, 0.0);
        ROS_INFO("SET Speed = %lf", set_speed_global);
    }
    else if (c == 'q'){
        ros::shutdown();
    }
  }
}

double speed2throttle_brake_PID_update(double set_speed, double atual_speed){
  
  double speed_error = set_speed - atual_speed;
  double normalized_value = normalize(30.0, 0.0, speed_error);
  
  normalized_value = std::min(normalized_value, 1.0);
  normalized_value = std::max(normalized_value, -1.0);
  
  return normalized_value;
}

void control_thread(int publish_rate)
{
  ros::Rate loop_rate(publish_rate);
  while (ros::ok())
  {
    double pid_throttle_brake_value = speed2throttle_brake_PID_update(set_speed_global, \
      car_info_global.speed_speedValue * 3.6); //* 3.6-> convert m/s to km/h
    //ROS_INFO("PID = %lf", pid_throttle_brake_value);
      ofstream outfile;
    control_pub_msg.throttle = std::max(0.0,pid_throttle_brake_value);
    control_pub_msg.brake = std::max(0.0, -pid_throttle_brake_value); 
    control_pub_msg.shift_gears = GEAR_FORWARD;
  
    if(ros::Time::now() > ros::TIME_MIN){

      pub_control_gazebo.publish(control_pub_msg);
      bag.write(pub_control_gazebo.getTopic(), ros::Time::now(), control_pub_msg);

      outfile.open("plots/steer_record.txt", ios::app);
      outfile << to_string(control_pub_msg.steer) << ", ";
      outfile.close();


      outfile.open("plots/time_record.txt",ios::app);
      outfile << to_string(ros::Time::now().toSec() ) << ", ";
      outfile.close();
    }
    loop_rate.sleep();
  }
}


int main(int argc, char **argv)
{
  if (argc < ARGC_MIN_REQUIRED) {
    help();
    ros::shutdown();
  }

  bag.open("test.bag", rosbag::bagmode::Write);

  std::stringstream node_name_ss;
  node_name_ss << "platooning_" << argv[1];
  ros::init(argc, argv, node_name_ss.str());
  car_name = node_name_ss.str();

  ros::NodeHandle n;


  

  std::stringstream car_name_leader_ss;
  car_name_leader_ss << "/" << argv[2] << "/";
  car_name_leader = car_name_leader_ss.str();

  std::stringstream rx_obu_topic;
  rx_obu_topic << "/" << argv[1] << "/RXNetwork";
  
  std::stringstream car_info_topic;
  car_info_topic << "/" << argv[1] << "/carINFO";

  std::stringstream pub_Control_topic_name_ss;
  pub_Control_topic_name_ss << "/" << argv[1] << "/prius";
        
  pub_control_gazebo              = n.advertise<prius_msgs::Control>(pub_Control_topic_name_ss.str(), 1);
  //ros::Subscriber sub_RX_obu      = n.subscribe(rx_obu_topic.str(), 10, RX_obu_Callback);
  ros::Subscriber sub_car_info    = n.subscribe(car_info_topic.str(), 10, car_info_Callback);

  boost::thread thread_keyboard(keyboard_thread, 10);
  boost::thread thread_control(control_thread, 10);

  ros::spin();

  bag.close();

  return 0;
}
