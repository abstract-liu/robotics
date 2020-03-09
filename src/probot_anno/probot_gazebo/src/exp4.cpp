#include <cmath>
#include <ros/forwards.h>
#include <ros/rate.h>
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include <vector>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "MotionFunction.h"
#include "MotionFunction.cpp"
#include <numeric>

const int RATE = 20;

int main(int argc, char **argv){
	//initialize robot
    ros::init(argc, argv, "exp4");
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


	std::vector<std::vector<float>> velTune = {
		{0,0.1,0.1,0},
		{0,0.1,0.1,0},
		{0,0.1,0.1,0},
		{0,0.1,0.1,0},
		{0,0.1,0.1,0},
		{0,0.1,0.1,0}
	};
	std::vector<std::vector<float>> a ={
		{0,0.06,0.06,0},
		{0,0.06,0.06,0},
		{0,0.06,0.06,0},
		{0,0.06,0.06,0},
		{0,0.06,0.06,0},
		{0,0.06,0.06,0}
	};
	std::vector<float> tf = {8,8,8};
	std::vector<std::vector<float>> pos = {
		{0.2289,0,0.454,1.57,0,0},
		{0.3,0.25,0.322,1.57,-1.57,0},
		{0.3,0.1,0.172,1.57,-1.57,0},
		{0.3,-0.1,0.122,1.57,-1.57,0}
	};

/*std::vector<std::vector<float>> velTune = {
		{0,0},
		{0,0},
		{0,0},
		{0,0},
		{0,0},
		{0,0}
	};
	std::vector<std::vector<float>> a ={
		{0,0},
		{0,0},
		{0,0},
		{0,0},
		{0,0},
		{0,0}
	};
	std::vector<float> tf = {8};
	std::vector<std::vector<float>> pos = {
		{0.2289,0,0.454,1.57,0,0},
		{0.3,0.25,0.322,1.57,-1.57,0},
	};
*/
	
	std::vector<std::vector<float>> velTable;
	velTable = GeneratePath(RATE, velTune, a, tf, pos);
	
	ros::Rate loopRate(RATE);
	int cnt = 0;
	while(ros::ok()){
	
	if(cnt > int(std::accumulate(tf.begin(),tf.end(),0))*RATE ){
		vel.data.at(0) = 0;
  		vel.data.at(1) = 0;
  		vel.data.at(2) = 0;
  		vel.data.at(3) = 0;
  		vel.data.at(4) = 0;
  		vel.data.at(5) = 0;
		vel_pub.publish(vel);
		break;
	}

	vel.data.at(0) = velTable[0][cnt];
  	vel.data.at(1) = velTable[1][cnt];
  	vel.data.at(2) = velTable[2][cnt];
  	vel.data.at(3) = velTable[3][cnt];
  	vel.data.at(4) = velTable[4][cnt];
  	vel.data.at(5) = velTable[5][cnt];
 
	vel_pub.publish(vel);
  	ROS_INFO_STREAM("published"<<vel.data.at(0));
	cnt += 1;
	loopRate.sleep();
	}	



}

