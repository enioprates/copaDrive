
//  ROSBAG Synchronized player
// BV (bffbv@isep.ipp.pt)

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <prius_msgs/Control.h>

#include "rosbag/player.h"
#include "rosbag/message_instance.h"
#include "rosbag/view.h"

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



int main(int argc, char **argv)
{
rosbag::Bag bag;

std::stringstream node_name_ss;
node_name_ss << "bag";
ros::init(argc, argv, node_name_ss.str());
ros::NodeHandle nh;

bag.open(argv[1], rosbag::bagmode::Read);
double gazebotime,rosbagtime, gazebotime_ns, rosbagtime_ns;
int timediff_after_ns,timediff_ns;
ros::Publisher topic_pub = nh.advertise<prius_msgs::Control>("/car1/prius", 1000);

std::vector<std::string> topics;
topics.push_back(std::string("/car1/prius"));


rosbag::View view(bag, rosbag::TopicQuery(topics));


ros::Time const& time = view.getBeginTime();
double rosbag_begin_time = time.toSec();
while(ros::ok()){
ros::spinOnce();
std::ofstream outfile;

do { 
     gazebotime = ros::Time::now().toSec();  
} while( gazebotime < rosbag_begin_time );

     

     foreach(rosbag::MessageInstance const m, view)
     {
     
     ros::Time const& time = m.getTime();
     rosbagtime = time.toSec();
     rosbagtime_ns = time.toNSec();

     gazebotime = ros::Time::now().toSec();
     gazebotime_ns = ros::Time::now().toNSec();



     if (rosbagtime_ns >= gazebotime_ns )
     {    
          //timediff_ns = int((rosbagtime_ns - gazebotime_ns));///1000000

          //std::cout << "timediff_ms:" << timediff_ns/1000000 << std::endl;

          while (rosbagtime_ns>gazebotime_ns){
               gazebotime = ros::Time::now().toSec();
               gazebotime_ns = ros::Time::now().toNSec();
          }


          //std::chrono::nanoseconds delay(timediff_ns);//timediff_ns 150

          //std::this_thread::sleep_for(delay);        

          prius_msgs::Control::ConstPtr s = m.instantiate<prius_msgs::Control>();
          topic_pub.publish(*s);

          //double ss = to_string(s);

         /* outfile.open("plots/steer_play.txt", std::ios::app);
          outfile << std::to_string(s.steer) << ", ";
          outfile.close();

          outfile.open("plots/time_play.txt", std::ios::app);
          outfile << std::to_string(ros::Time::now().toSec() ) << ", ";
          outfile.close();
          */
          //gazebotime_ns = ros::Time::now().toNSec();

          //timediff_after_ns = int((rosbagtime_ns - gazebotime_ns));
          //std::cout << "timediff_after_ms:" << timediff_after_ns/1000000 << std::endl;

          //std::cout << "______________________" << std::endl;


          
          

     } else {

          std::this_thread::sleep_for(std::chrono::microseconds(100));
          std::cout << "gazebo after rosbag!"<< std::endl;

     }

     }

bag.close();
}

}

