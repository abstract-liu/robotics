#include <ros/rate.h>
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "MotionFunction.h"
#include "MotionFunction.cpp"

const int RATE = 20;
int main(int argc, char **argv) {

    ros::init(argc, argv, "exp5");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);
 

	std::vector<float> pos1 = {0.2289, 0, 0.454, 1.57, 0, 0};
	std::vector<float> pos2 = {0.2289, 0, 0.454, 1.4, 0.5, 0};

	std::vector<std::vector<float>> velTable;

	velTable = LineaerEuler(RATE, 120, pos1, pos2);

	std_msgs::Float64MultiArray vel;
	vel.data.push_back(0);
	vel.data.push_back(0);
	vel.data.push_back(0);
	vel.data.push_back(0);
	vel.data.push_back(0);
	vel.data.push_back(0);
	sleep(1);

	ros::Rate loopRate(RATE);
	int cnt = 0;
	while(ros::ok()){
	
	if(cnt == velTable[0].size()){
		break;
	}

	vel.data.at(0) = velTable[0][cnt];
  	vel.data.at(1) = velTable[1][cnt];
  	vel.data.at(2) = velTable[2][cnt];
  	vel.data.at(3) = velTable[3][cnt];
  	vel.data.at(4) = velTable[4][cnt];
  	vel.data.at(5) = velTable[5][cnt];
	vel_pub.publish(vel);
	cnt += 1;
	loopRate.sleep();
	}	


		

}
