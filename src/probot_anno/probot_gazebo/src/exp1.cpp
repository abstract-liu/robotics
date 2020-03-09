#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"

int main(int argc, char **argv){

    bool ret;
    //节点初始化
    ros::init(argc, argv, "test");
    //创建节点句柄对象
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    ros::Publisher pos_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);

    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    sleep(1);

    float target[3][6] = {{0.927,-0.687,-0.396,0,1.083,0.927},
	    {0.322,-0.855,-0.021,0,0.877,0.322},
	    {-0.322,-0.636,-0.011,0,0.647,-0.322}};
    int action_num = 0;

    while(true){
	    std::cin >> action_num;
	switch(action_num){
	    case 1:{
		 init_pos.data.at(0) = target[0][0];
   		 init_pos.data.at(1) = target[0][1];
   		 init_pos.data.at(2) = target[0][2];
   		 init_pos.data.at(3) = target[0][3];
   		 init_pos.data.at(4) = target[0][4];
   		 init_pos.data.at(5) = target[0][5];
		   
   		 pos_pub.publish(init_pos);
		 break;}
	     case 2:{
                 init_pos.data.at(0) = target[1][0];
                 init_pos.data.at(1) = target[1][1];
                 init_pos.data.at(2) = target[1][2];
                 init_pos.data.at(3) = target[1][3];
                 init_pos.data.at(4) = target[1][4];
                 init_pos.data.at(5) = target[1][5];

                 pos_pub.publish(init_pos);
		 break;}
	     case 3:{
                 init_pos.data.at(0) = target[2][0];
                 init_pos.data.at(1) = target[2][1];
                 init_pos.data.at(2) = target[2][2];
                 init_pos.data.at(3) = target[2][3];
                 init_pos.data.at(4) = target[2][4];
                 init_pos.data.at(5) = target[2][5];

                 pos_pub.publish(init_pos);
		 break;}
		 default: std::cout <<"ERROR!"<< endl;	
			    }
   	action_num = 0;
    }


}
