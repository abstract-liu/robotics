#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"

const float PI = 3.1415926;

using namespace std;

int main(int argc, char **argv){
  bool ret;
  ros::init(argc, argv, "velocity_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  ROS_INFO_STREAM("start");

  ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 100);


  std_msgs::Float64MultiArray init_vel;
  init_vel.data.push_back(0);
  init_vel.data.push_back(0);
  init_vel.data.push_back(0);
  init_vel.data.push_back(0);
  init_vel.data.push_back(0);
  init_vel.data.push_back(0);
  sleep(1);

  init_vel.data.at(0) = 1;
  init_vel.data.at(1) = 0.00;
  init_vel.data.at(2) = 0.00;
  init_vel.data.at(3) = 0.00;
  init_vel.data.at(4) = 0.00;
  init_vel.data.at(5) = 0.00;
 
  vel_pub.publish(init_vel);
  ROS_INFO_STREAM("published");
  
}
