#include <cmath>
#include <ros/forwards.h>
#include <ros/rate.h>
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include <Eigen/Dense>
#include "MotionFunction.h"
#include "MotionFunction.cpp"

const int RATE = 20;

int main(int argc, char **argv){
	//initialize robot
    ros::init(argc, argv, "exp2");
  	ros::NodeHandle node_handle;
  	ros::AsyncSpinner spinner(1);
  	ROS_INFO_STREAM("start");

 	 ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);


  	std_msgs::Float64MultiArray vel;
  	vel.data.push_back(0);
  	vel.data.push_back(0);
  	vel.data.push_back(0);
  	vel.data.push_back(0);
  	vel.data.push_back(0);
  	vel.data.push_back(0);
  	sleep(1);

	//calculate velocity
	std::vector<std::vector<float>> velocityTab(12*RATE, std::vector<float>(6,0));
	ComputeVelTab(velocityTab, RATE);
	
	//create rate_loop
	ros::Rate loopRate(RATE);
	int cnt = 0;
	while(ros::ok()){
	
	if(cnt>12*RATE - 1){
		vel.data.at(0) = 0;
  		vel.data.at(1) = 0;
  		vel.data.at(2) = 0;
  		vel.data.at(3) = 0;
  		vel.data.at(4) = 0;
  		vel.data.at(5) = 0;
		vel_pub.publish(vel);
		break;
	}

	vel.data.at(0) = velocityTab[cnt][0];
  	vel.data.at(1) = velocityTab[cnt][1];
  	vel.data.at(2) = velocityTab[cnt][2];
  	vel.data.at(3) = velocityTab[cnt][3];
  	vel.data.at(4) = velocityTab[cnt][4];
  	vel.data.at(5) = velocityTab[cnt][5];
 
	vel_pub.publish(vel);
  	ROS_INFO_STREAM("published"<<cnt);
	cnt += 1;
	loopRate.sleep();
	}	

}

