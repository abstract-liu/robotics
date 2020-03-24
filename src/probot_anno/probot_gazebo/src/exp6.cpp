#include <ros/datatypes.h>
#include <ros/init.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
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
#include "sensor_msgs/JointState.h"


double theta[6] = {0,0,0,0,0,0};

void JointCallback(const sensor_msgs::JointStateConstPtr& msg){

	theta[0] = msg->position[0];
	theta[1] = msg->position[1];
	theta[2] = msg->position[2];
	theta[3] = msg->position[3];
	theta[4] = msg->position[4];
	theta[5] = msg->position[5];
	ROS_INFO_STREAM(theta[0]);
}



const int RATE = 1000;
int main(int argc, char **argv) {

    ros::init(argc, argv, "exp6");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);
	ros::Subscriber sub = node_handle.subscribe("probot_anno/joint_states", 1, JointCallback);
 
	std::vector<std::vector<double>> pos1 = {
		{0.2289, 0.26},
		{0, 0.15},
		{0.454, 0.08}
	};

	std::vector<std::vector<double>> vel1 = {
		{0, 0},
		{0, 0},
		{0, 0}
	};
	
	std::vector<std::vector<double>> a1 = {
		{0, 0},
		{0, 0},
		{0, 0}
	};


	std::vector<std::vector<double>> pos2 = {
		{0.26, 0.38, 0.28},
		{0.15, 0, -0.24},
		{0.08, 0.24, 0.08}
	};

	std::vector<std::vector<double>> vel2 = {
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0}
	};
	
	std::vector<std::vector<double>> a2 = {
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0}
	};


	std::vector<std::vector<double>> pos3 = {
		{0.28, 0.38, 0.26},
		{-0.24, 0, 0.15},
		{0.08, 0.24, 0.08}
	};

	std::vector<std::vector<double>> vel3 = {
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0}
	};
	
	std::vector<std::vector<double>> a3 = {
		{0, 0, 0},
		{0, 0, 0},
		{0, 0, 0}
	};

	std::vector<double> tf1 = {2.7};
	std::vector<double> tf2 = {2.7, 2.7};
	

	std::vector<std::vector<double>> velTable1, velTable2, velTable3;
	
	velTable1 = HitBell(RATE, theta,  vel1, a1, tf1, pos1);
	ROS_INFO_STREAM(theta[0]);
	velTable2 = HitBell(RATE, theta, vel2, a2, tf2, pos2);
	velTable3 = HitBell(RATE, theta, vel3, a3, tf2, pos3);
	ROS_INFO_STREAM(theta[0]);


	std_msgs::Float64MultiArray vel;
	vel.data.push_back(0);
	vel.data.push_back(0);
	vel.data.push_back(0);
	vel.data.push_back(0);
	vel.data.push_back(0);
	vel.data.push_back(0);

	ros::Rate loopRate(RATE);
	int cnt = 0, gpNum = 0, count = 0;

	ROS_INFO_STREAM("start");
	while(ros::ok()){
	
	if(cnt == velTable1[0].size() && gpNum == 0){
		cnt = 0; 
		gpNum = 1;
		ros::spinOnce();
	}
	if(cnt == velTable2[0].size() && gpNum == 1){
		cnt = 0;
		gpNum = 2;
		count += 1;
		ROS_INFO_STREAM(count);
	}
	if(cnt == velTable3[0].size() && gpNum == 2){
		cnt = 0;
		gpNum = 1;
		ros::spinOnce();
	}

	

	if(gpNum == 0){
		vel.data.at(0) = velTable1[0][cnt];
  		vel.data.at(1) = velTable1[1][cnt];
  		vel.data.at(2) = velTable1[2][cnt];
  		vel.data.at(3) = velTable1[3][cnt];
  		vel.data.at(4) = velTable1[4][cnt];
  		vel.data.at(5) = velTable1[5][cnt];
	}

	if(gpNum == 1){
		vel.data.at(0) = velTable2[0][cnt];
  		vel.data.at(1) = velTable2[1][cnt];
  		vel.data.at(2) = velTable2[2][cnt];
  		vel.data.at(3) = velTable2[3][cnt];
  		vel.data.at(4) = velTable2[4][cnt];
  		vel.data.at(5) = velTable2[5][cnt];
	}
	
	if(gpNum == 2){
		vel.data.at(0) = velTable3[0][cnt];
  		vel.data.at(1) = velTable3[1][cnt];
  		vel.data.at(2) = velTable3[2][cnt];
  		vel.data.at(3) = velTable3[3][cnt];
  		vel.data.at(4) = velTable3[4][cnt];
  		vel.data.at(5) = velTable3[5][cnt];
	}
	

	
	vel_pub.publish(vel);
	cnt += 1;
	loopRate.sleep();
	}	


		

}
