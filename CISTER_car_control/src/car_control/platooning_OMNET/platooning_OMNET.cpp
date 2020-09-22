// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros_its_msgs/CAM_simplified.h>
#include <ros_its_msgs/platoon_dist.h>
#include <prius_msgs/Control.h>
#include "ros/console.h"
#include <sstream>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <cmath>
#include <string>
#include <iostream>
#include <cstdio>
#include <fstream>
#include <chrono>

#include <ctime>
#define BUFSIZE 100

//Global vars
using namespace std;
std::string car_name;
std::string car_name_leader;
uint32_t car_id;

double KPs=0.3;//2 0.3
double KIs=0.0005;//0.005 0.0005
double KDs=0; //2 0

double PID;

double KPstr=1;//0.7
double KIstr=0.06; //0.06
double KDstr=0.0; //0.1

bool BEARING_ON=true;
bool W_OBU=false;
double PreviousError;
double integral;
double integral2;
double PE2;
double Tempo1;
double dt;
double aa,leadersteer,leaderangle,leaderspeed,leaderTspeed;
double steerResult;
double t1,dtt,Tempo11;
double ddt;
double Fnormal;
double controlled=1;
bool ControlFlag,control_ready;
double GPSREACH_dist;

typedef struct
{
        double x ;
        double y ;
} POSITION ;

struct MSG 
{
double LV_angles;
double LV_steers;
double LV_speeds;
double LV_current_speeds;
double LV_X;
double LV_Y;
double L_Throttle;
double L_Brake;
}Vmessage[BUFSIZE];

//MSG Vmessage[BUFSIZE];
MSG * Vmessageptr[BUFSIZE];



double angle;
double steer;
double speed;
double current_speed;
double distance1;// distance between the leader and follower 
double old_angle;
double tmp,ft;  
POSITION position; 
//define initial positions
double x_initial,y_initial;
// Received data 
POSITION LV_positions[5000];
double xr,yr,x_ref,y_ref;
/*double LV_angles[5000];
double LV_steers[5000];
double LV_speeds[5000];
double LV_current_speeds[5000];
 
double LV_X[5000];
double LV_Y[5000];  
double L_Throttle[5000];
double L_Brake[5000];*/
int k,k2;
string robotName;
bool following_started=true;
bool leader_start=false;
int ptf=0, ptl=0;
double status=0;
double GPSREACH_dist1,GPSREACH_lowest;
/*ptf pointer to last computed information by the follower 
ptl pointer to the last information received from leader  */

double Throttle,Brake,SpeedDiff;


bool computed;   
double control;           
        
#define ARGC_MIN_REQUIRED 3 // minimum required number of parameters

#define GPSREACH 3 //3->SD=8 5.5->SD=15
#define BEARING_THRESHOLD 0.15


#define SAFETY_DISTANCE 8
#define INITIAL_SPEED 30

ros::Publisher pub_control_gazebo;
ros::Publisher pub_platoon_dist;

double getTime(){
       // ROS_INFO("Time:%f", ros::Time::now().toSec());
        return ros::Time::now().toSec();
}

double normalize(double x_max, double x_min, double value){
  return (value - x_min) / (x_max - x_min);
}

double speed2throttle_brake_PID_update(double set_speed, double atual_speed){
  double normalized_value;
  double speed_error = set_speed - atual_speed;
  
  if(speed_error>0)
  normalized_value = normalize(30.0, 0.0, speed_error);
  else 
  normalized_value = normalize(30.0, 0.0, speed_error);
  
  normalized_value = std::min(normalized_value, 1.0);
  normalized_value = std::max(normalized_value, -1.0);
  
  return normalized_value;
}


double PIDsetSafeDistance(double distancePID,double safeDistance,double speed1,double speed2,double currentSpeed,double oldValue,double integral, double dt)
{
  double Vfollowed;
  if(speed1==speed2){
    Vfollowed=speed1;
  }else Vfollowed=currentSpeed;
  
 // Vfollowed=currentSpeed;
  Vfollowed=speed1;
 
  double result=0.0;
  
  double err = (distancePID) - safeDistance;
  double diff= (err-oldValue)/dt;
  

  result= ((KPs * err) + Vfollowed) + (KIs * integral) + (KDs * diff);
  return result;
}

double PIDsetSteer(double setpoint,double vehicle_angle, double oldValue, double integral, double dt, double steer_f, double steer_l,double bearing_angle)
{
  double rs; // the output of PID
  double error;
  
  double anglefix=0;
  double headingfix=0;
  double steerfix=0;
  
  bool sig,sig1;
  double headingfixdeg=0;
  double anglefixdeg=0;
  
  double bearingdeg=(bearing_angle*180/M_PI)+90;
  double spdeg=setpoint*180/M_PI;
  double vadeg=vehicle_angle*180/M_PI;// rad -> º
 
      //steerResult = abs(steer_l - steer_f);
      steerResult = bearingdeg;

    if (spdeg < 0){
        spdeg=360+spdeg;
        }//0º-360º
  
    if (vadeg < 0){
        vadeg=360+vadeg;
        } //0º-360º
  
  if (bearingdeg < 0){
        bearingdeg=360+bearingdeg;
        }//0º-360º
  

  

  if (vadeg>spdeg && (signbit(vadeg)==signbit(spdeg))){
        if (abs(vadeg) > 300 && abs(spdeg) < 100)
        {
        headingfixdeg=(360-vadeg)+spdeg;
        }
        else
        {
        headingfixdeg=vadeg-spdeg;
        }
        sig=false;
  }
  
  if (vadeg<spdeg && (signbit(vadeg)==signbit(spdeg))){

        if (abs(spdeg) > 300 && abs(vadeg) < 100)
        {
        headingfixdeg=(360-spdeg)+vadeg;
        }
        else
        {
        headingfixdeg=(spdeg-vadeg);
        }
        sig=true;
 }
  
  if (sig==true){
        headingfix=(headingfixdeg*M_PI)/180;} //rad+
  
  if (sig==false){
        headingfix=-(headingfixdeg*M_PI)/180;}//rad-
  
  
  

  
  if (vadeg>bearingdeg && (signbit(vadeg)==signbit(bearingdeg)))
  {
        if (abs(vadeg) > 300 && abs(bearingdeg) < 100)
        {
        anglefixdeg=(360-vadeg)+bearingdeg;
        }
        else
        {
            anglefixdeg=vadeg-bearingdeg;//0-360
        }

        sig1=false;
       // ROS_INFO("!");
 }//esquerda
  
  if (vadeg<bearingdeg && (signbit(vadeg)==signbit(bearingdeg)))
  {

        if (abs(bearingdeg) > 300 && abs(vadeg) < 100)
        {
        anglefixdeg=(360-bearingdeg)+vadeg;
        }
        else
        {
            anglefixdeg=(bearingdeg-vadeg);//0-360
        }
        
        sig1=true;
 }// direita
  
  if (sig1==true){
        anglefix=(anglefixdeg*M_PI)/180;} //rad+
  
  if (sig1==false){
        anglefix=-(anglefixdeg*M_PI)/180;}//rad-
  
  
  
  if (abs(anglefix)<BEARING_THRESHOLD  || abs(anglefix)>15){
      anglefix=0;
      }
          
  if (signbit(setpoint)!=signbit(vehicle_angle)){
      headingfix=0;
      headingfixdeg=0;
      }

  //steerfix=steer_l-steer_f; //SteerOFF
  
  if(BEARING_ON==false | signbit(vehicle_angle)!=signbit(bearing_angle)){
      anglefix=0; //BearingOFF
      anglefixdeg=0;
      }

  error = anglefix + headingfix + steerfix;

  //ROS_INFO("setpoint:%f",setpoint);

  ///ROS_INFO("spdeg:%f",spdeg);

  //ROS_INFO("vehicle_angle:%f",vehicle_angle);

  ///ROS_INFO("vadeg:%f",vadeg);
  ///ROS_INFO("bearingdeg:%f",bearingdeg);
  ///ROS_INFO("headingfixdeg:%f",headingfixdeg);

  //ROS_INFO("headingfix:%f(rad)",headingfix);

  ///ROS_INFO("anglefixdeg:%f)",anglefixdeg);

  //ROS_INFO("anglefix:%f(rad)",anglefix);

  ///ROS_INFO("error:%f(rad)",error);

  double diff = (error - oldValue)/dt; 
    
  rs = KPstr * error +  KIstr * integral + KDstr * diff;
  
  //ROS_INFO("rs:%f",rs);
      //steerResult=rs;
  return rs;
}
   
   
   void normalMode()
      {
      //double PreviousError;
      double new_angle=0.0;
      prius_msgs::Control pub_control_msg;
      ros_its_msgs::platoon_dist pub_platoon_dist_msg;
      // ROS_INFO("x_ref:%f",x_ref);
      // ROS_INFO("position.x:%f",position.x);
      if (control_ready==false){
            Throttle=0.5;
         
            if(car_name=="platooning_car3"){
                  Throttle=0.5;
            }
       
            pub_control_msg.throttle=Throttle;
            pub_control_gazebo.publish(pub_control_msg);
            //ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!");
      }
       
      GPSREACH_dist=(sqrt(pow(x_ref-position.x,2)+pow(y_ref-position.y,2)));
      //ROS_INFO("%s", car_name.c_str());
       
      if(car_name=="platooning_car2"||car_name=="platooning_car3"){
           //ROS_INFO("GPSREACH_dist:%f  ptf:%d",GPSREACH_dist,ptf);
           //ROS_INFO("x_ref:%f | y_ref:%f",x_ref,y_ref);
           //ROS_INFO("XFollower:%f | YFollower:%f",position.x,position.y);
      }
                
                        
      if(GPSREACH_dist<GPSREACH)
      {
            control_ready=true;

            if(ptf!=BUFSIZE-2){
                  x_ref=Vmessage[ptf+1].LV_X;
                  y_ref=Vmessage[ptf+1].LV_Y;
            } 
            else{
                  x_ref=Vmessage[0].LV_X;
                  y_ref=Vmessage[0].LV_Y;
            }

            computed=true;
             
            dt=getTime()-Tempo1;
            Tempo1= getTime();

            //cout << "dt = " << dt << endl;
           // ROS_INFO("dt:%f",dt);

            integral2+= (distance1-SAFETY_DISTANCE )* dt;
                        
            if(ptf!=BUFSIZE)
            PID=PIDsetSafeDistance(distance1,SAFETY_DISTANCE,Vmessage[ptf].LV_speeds,Vmessage[(ptf)+1].LV_speeds,Vmessage[ptf].LV_current_speeds,PE2,2,dt);
            else 
            PID=PIDsetSafeDistance(distance1,SAFETY_DISTANCE,Vmessage[ptf].LV_speeds,Vmessage[0].LV_speeds,Vmessage[ptf].LV_current_speeds,PE2,2,dt);

            PE2= distance1-SAFETY_DISTANCE ;
			
	      pub_platoon_dist_msg.car_name = car_name;
		pub_platoon_dist_msg.Ref_distance = SAFETY_DISTANCE;
		pub_platoon_dist_msg.leader_distance = distance1;
		pub_platoon_dist.publish(pub_platoon_dist_msg);


                        
                        
                        
                        
            //SpeedDiff=PID-Vmessage[ptf].LV_current_speeds;
                        
                       
									
		double pid_throttle_brake_value = speed2throttle_brake_PID_update(PID*3.6, speed * 3.6);
		Throttle = std::max(0.0,pid_throttle_brake_value);
		Brake = std::max(0.0, -pid_throttle_brake_value);

            double bearing_angle=atan2((y_ref-position.y),(x_ref-position.x));// Bearing
                        
            integral+=(Vmessage[ptf].LV_angles-angle)*dt; //Heading only
                         
            if (integral < 10 && integral > -10){
                  integral=0;
            }
                        
            if( Vmessage[ptf].LV_current_speeds>50){
                  KPstr=1.5;
            }
                        
            double q=0;
                        
            if(signbit(position.x)==false && signbit(position.y)==false){
                  q=1;
            }
            if(signbit(position.x)==true && signbit(position.y)==false){
                  q=2;
            }
            if(signbit(position.x)==true && signbit(position.y)==true){
                  q=3;
            }
            if(signbit(position.x)==false && signbit(position.y)==true){
                  q=4;
            }
                        
            new_angle=PIDsetSteer(Vmessage[ptf].LV_angles,angle,PreviousError,integral,dt, steer, Vmessage[ptf].LV_steers,bearing_angle);
                        
                        
                        
            if (signbit(Vmessage[ptf].LV_angles) == signbit(angle)){
                  PreviousError=Vmessage[ptf].LV_angles-angle;
            }
                  
                  
            if ((signbit(Vmessage[ptf].LV_angles) != signbit(angle)))
            {
                   
                  if((-M_PI/2)>Vmessage[ptf].LV_angles && Vmessage[ptf].LV_angles>(-M_PI))//3q
                  {
                        PreviousError=M_PI-angle+(Vmessage[ptf].LV_angles+M_PI);
                  }
                          
                          
                  if((-M_PI/2)<Vmessage[ptf].LV_angles && Vmessage[ptf].LV_angles<0)//4q
                  {
                        PreviousError=-angle+Vmessage[ptf].LV_angles;
                  }
                          
                  if((M_PI/2)>Vmessage[ptf].LV_angles && Vmessage[ptf].LV_angles>0)//1q
                  {
                        PreviousError=-angle+Vmessage[ptf].LV_angles;
                  }
                          
                  if((M_PI/2)<Vmessage[ptf].LV_angles && Vmessage[ptf].LV_angles<(M_PI))//2q
                  {
                        PreviousError=(M_PI+angle)+(M_PI-Vmessage[ptf].LV_angles);
                  }
                  

            } 
                             
            if (abs(new_angle)<0.8727)       
                  steer=new_angle;
            else if(new_angle<-0.8727)
                  steer=-0.8727;
            else if (new_angle>0.8727)
                  steer=0.8727;
                        
            double heading=angle *180 / M_PI ;
            double bearing=bearing_angle*180/M_PI;
                         
            if (heading < 0){
                  heading=360+heading;
            }
                              
            if (bearing < 0){
                  bearing=360+bearing;
            }

                                                
            //ROS_INFO("distance:%f(m)",distance1);
            //ROS_INFO("Throttle:%f",Throttle);
            // ROS_INFO("Brake:%f",Brake);
            //ROS_INFO("LSpeed:%f(m/s)",LV_speeds[ptf]);
            //ROS_INFO("SpeedDiff:%f(m/s)",SpeedDiff);
                        
            // ROS_INFO("setpoint:%f",LV_angles[ptf]);
            // ROS_INFO("vehicle_angle:%f",angle);
            //ROS_INFO("Steer:%f(rad)",new_angle);
                        
                        
            // ROS_INFO("k:%d",k);
            //ROS_INFO("ptl:%d",ptl);
            //ROS_INFO("ptf:%d",ptf);
            
                        
            old_angle=angle;
                          

            control=ptf;//control=indice da ultima mensagem controlada
            k2++;
            if(k2==BUFSIZE-1)
                  k2=0;
                        
            pub_control_msg.throttle=Throttle;
            pub_control_msg.brake=Brake;
            pub_control_msg.shift_gears=2;//FORWARD
                        
            //if(abs((steer/0.8727)-Vmessage[ptf].LV_steers<1))
            pub_control_msg.steer=steer/0.8727;

           // steerResult=pub_control_msg.steer;
                        
                        
            //ROS_INFO("control_msg.Steer:%f(rad)", pub_control_msg.steer);
            pub_control_gazebo.publish(pub_control_msg);
                //}
                
      }
                        
       return;
      }


    void run() {
    prius_msgs::Control pub_control_msg;
    ros_its_msgs::platoon_dist pub_platoon_dist_msg;
      //setCruisingSpeed(INITIAL_SPEED);// Initiate the speed ??
      computed=true;   
      ptf=k2;
      ptl=k;
        //ROS_INFO("RUN");
       // getTime();
        static int i=0;
        
         if(ControlFlag==true) //0.05s ??
                  { 
                  
                  
                 

           //getVehicleInfo(); // Get vehicle informations ??

                   
               // ROS_INFO("S_PTL:%lf",LV_speeds[ptl-1]);
                
            if(!leader_start)
            {

              //cout << "speed:" << Vmessage[ptl-1].LV_speeds << endl;
              //if(LV_speeds[ptl-1]>2.00)
              if(Vmessage[ptl-1].LV_speeds>=3)
              {
              ROS_INFO("I'm %s and I heard:Leader started", car_name.c_str());
                leader_start=true;
                control_ready=false;
                Throttle=0.8;
                k2=k;
                x_ref=Vmessage[ptl-1].LV_X;
                y_ref=Vmessage[ptl-1].LV_Y;
                
               //Throttle=L_Throttle[ptl-1]+0.00;
                pub_control_msg.throttle=Throttle;
                pub_control_gazebo.publish(pub_control_msg);
              }
            }
            
            if(leader_start)
            {
              
              normalMode();// Normal Mode Behavior
                  if ( car_name == "car2"){
                	std::ofstream myfile;
	            myfile.open("PlatooningCar2Longdistance.csv", std::ofstream::out | std::ofstream::app);
	            myfile << distance1  << std::endl;
	            myfile.close();
                  }  

                  if ( car_name == "car2"){
                	std::ofstream myfile;
	            myfile.open("PlatooningCar2Bearing.csv", std::ofstream::out | std::ofstream::app);
	            myfile << steerResult  << std::endl;
	            myfile.close();
                  }  
     
                
            } 
                 
            
                ControlFlag=false;
         }                  
            
 ++i;
         
            
            
      }
      


      
    

int help() {
	ROS_WARN("Missing [car_name] or [car_name_leader]!");
	ROS_WARN("Usage: rosrun car_demo control_test [car_name] [car_name_leader]");
	return 1;
}

void RX_obu_Callback(const ros_its_msgs::CAM_simplified::ConstPtr& msg){
	
	
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
      // getTime();
      //ROS_INFO("RX_obu");

        //if(abs((msg->latitude)-LV_X[ptl])>10){
        //if(msg->gasPedalPercent_Value>=0.1 || msg->speed_speedValue>=2){
                if(msg->car_name == car_name_leader){
                //getTime();
                if (k==BUFSIZE-1)
                k=0;
                
                
                xr = msg->latitude;
                yr = msg->longitude;
                Vmessage[k].LV_angles=msg->heading_headingValue; //-M_PI/2
                Vmessage[k].LV_steers=msg->steeringWheelAngle_steeringWheelAngleValue;
                Vmessage[k].LV_speeds=msg->speed_speedValue;
                Vmessage[k].LV_current_speeds=msg->speed_speedValue;
                Vmessage[k].LV_X=msg->latitude;
                Vmessage[k].LV_Y=msg->longitude;

                //ROS_INFO("Topic from: ", msg->car_name);
                //ROS_INFO("________________");
                //Vmessage[k].L_Throttle=msg->gasPedalPercent_Value;
                //Vmessage[k].L_Brake=msg->brakePedalPercent_Value;

            /*
                ROS_INFO("lat %f",msg->latitude);
                ROS_INFO("long %f",msg->longitude);
                ROS_INFO("heading %f",msg->heading_headingValue);
                ROS_INFO("steer %f",msg->steeringWheelAngle_steeringWheelAngleValue);
                ROS_INFO("speed %f",msg->speed_speedValue);
                ROS_INFO("________________");

                */

                k++;
 
                //if (k==0){
                  //      x_ref=xr;
                    //    y_ref=yr;
                //}
                  if ( car_name == "car2"){
                	std::ofstream myfile;
	            myfile.open("Car2CAMReception.csv", std::ofstream::out | std::ofstream::app);
                  auto t0 = std::chrono::high_resolution_clock::now();        
    			auto nanosec = t0.time_since_epoch();
	            myfile << nanosec.count()  << std::endl;
	            myfile.close();
                  }


                
                
                
                
                
        }
        //}
	//if(msg->car_name == car_name_leader){
		//ROS_INFO("I'm %s and I heard latitude: [%f] from Leader from RX_obu_Callback", car_name.c_str(), msg->latitude);
		//ROS_INFO("I'm %s and I heard longitude: [%f] from Leader from RX_obu_Callback", car_name.c_str(), msg->longitude);
	
	        //prius_msgs::Control pub_control_msg;
	        //outputs (prius_msgs::Control)
                //pub_control_msg.throttle (float64) Range 0 to 1, 1 is max throttle
                //pub_control_msg.brake (float64) Range 0 to 1, 1 is max brake
                //pub_control_msg.steer (float64) Range -1 to +1, +1 is maximum left turn
                //pub_control_msg.shift_gears (uint8) NO_COMMAND=0 NEUTRAL=1 FORWARD=2 REVERSE=3
	        //pub_control_gazebo.publish(pub_control_msg);
	        
	return;
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


        //getTime();
        
        
        angle = msg->heading_headingValue;//-M_PI/2
        position.x= msg->latitude ;
        position.y=msg->longitude;  
        speed=msg->speed_speedValue;     
        current_speed = msg->speed_speedValue; 
        steer = msg->steeringWheelAngle_steeringWheelAngleValue;
        
        //ROS_INFO("xr:%f",xr);
        //ROS_INFO("position.x:%f",position.x);
        
        distance1=sqrt(pow(xr-position.x,2)+pow(yr-position.y,2));
        
        ControlFlag=true;// 0.05s ??
	
        
	return;
}




int main(int argc, char **argv)
{
	if (argc < ARGC_MIN_REQUIRED) {
		help();
		ros::shutdown();
	}
	
        
//Vmessageptr=&Vmessage; 

	std::stringstream node_name_ss;
	//node_name_ss << "platooning_" << argv[1];
	node_name_ss << argv[1]; 
	ros::init(argc, argv, node_name_ss.str());
	car_name = node_name_ss.str();

	ros::NodeHandle n;

	std::stringstream car_name_leader_ss;
	car_name_leader_ss << "/" << argv[2] << "/";
	car_name_leader = car_name_leader_ss.str();

	std::stringstream rx_obu_topic;



    
      rx_obu_topic << "/" << argv[1] << "/RXNetwork_OMNET";
      cout << "subscribe to: " << rx_obu_topic.str() << endl;

	std::stringstream car_info_topic;
	car_info_topic << "/" << argv[1] << "/carINFO";

	std::stringstream pub_Control_topic_name_ss;
	pub_Control_topic_name_ss << "/" << argv[1] << "/prius";
        
	std::stringstream pub_platoon_dist_topic_name_ss;
	pub_platoon_dist_topic_name_ss << "/Platoon_dist";
	


      pub_control_gazebo 		= n.advertise<prius_msgs::Control>(pub_Control_topic_name_ss.str(), 10);
	pub_platoon_dist 		= n.advertise<ros_its_msgs::platoon_dist>(pub_platoon_dist_topic_name_ss.str(), 10);
	ros::Subscriber sub_RX_obu	= n.subscribe(rx_obu_topic.str(), 10, RX_obu_Callback);
	ros::Subscriber sub_car_info	= n.subscribe(car_info_topic.str(), 10, car_info_Callback);

        
        
        while(ros::ok()){
        ros::spinOnce();
        run();
        }
        
	//ros::spin();

	return 0;
}
