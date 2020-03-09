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


int main(int argc, char **argv) {

    ros::init(argc, argv, "exp1_b");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    ros::Publisher pos_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);
    
	std::vector<std::vector<float>> pos = {{0.2, 0.2, 0.2007, 1.57, -1.57, 0},
	{0.15, 0.2, 0.2007, 0, 0, 0},
	{0.3, 0, 0.122, 1.57, 0, 0},
	{0.3, 0.25, 0.322, 1.57, -1.57, 0}
	};

    int group_num = 0;
    while(true){
		std::cin >> group_num;
		if (group_num == 0) continue;
		else if(group_num == -1 ) break;	
		else{
			std::vector<std::vector<double>> angles;
			angles = BackwardAngle(pos[group_num-1]);

    		std_msgs::Float64MultiArray pos;
   			pos.data.push_back(0);
    		pos.data.push_back(0);
    		pos.data.push_back(0);
    		pos.data.push_back(0);
    		pos.data.push_back(0);
    		pos.data.push_back(0);
    		sleep(1);

    		pos.data.at(0) = angles[0][0];
    		pos.data.at(1) = angles[0][1];
    		pos.data.at(2) = angles[0][2];
    		pos.data.at(3) = angles[0][3];
    		pos.data.at(4) = angles[0][4];
    		pos.data.at(5) = angles[0][5];

    		pos_pub.publish(pos);
    		ROS_INFO_STREAM("published");
    		group_num = 0;
			}
    }
}
