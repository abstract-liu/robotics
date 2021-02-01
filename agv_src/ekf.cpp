#include "ros/ros.h"
#include "ros/console.h"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <stdio.h>
#include <numeric>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <vector>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

class ekf{

public:
    ekf(ros::NodeHandle &n);
	~ekf();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta; 
    // match threshold;
    float match_th;
    // status
    VectorXd status;
    MatrixXd covariance;

	VectorXd estimated_status;
	MatrixXd estimated_covariance;

    // noise R
    MatrixXd noise_R;
    // noise Q
    MatrixXd noise_Q;
    // landmark num
    int landMark_num;
    // noises
    float noise_motion, noise_measure;
	//translation and delta
	bool isFirstScan;
	
	MatrixXd landMark;
	MatrixXd last_landMark;

	int new_point;
    
    // init all 
    void initAll();
    // predict phase
    void predict(nav_msgs::Odometry odom);
    // update phase
    void update(visualization_msgs::MarkerArray input);
    // landMarks to XY matrix
    Eigen::MatrixXd landMarksToXY(visualization_msgs::MarkerArray input);
    // landMarks to r-phi matrix
    Vector2d cartesianToPolar(double x, double y);
    // update feature map
    void updateFeatureMap(Eigen::MatrixXd newFeatures);
    // angle normalization
    double angleNorm(double angle);
    // calc 2D distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // find nearest map points
    int findNearestMap(Vector2d point);


    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    ros::Subscriber icpOdom_sub;
	ros::Publisher landMark_pub;
	ros::Publisher odom_pub;
    tf::TransformBroadcaster ekf_broadcaster;
    void publishResult();
};

ekf::~ekf()
{}

ekf::ekf(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/ekf/robot_x", robot_x);
	n.getParam("/ekf/robot_y", robot_y);
	n.getParam("/ekf/robot_theta", robot_theta);

    n.getParam("/ekf/match_th", match_th);
    n.getParam("/ekf/landMark_num", landMark_num);
    n.getParam("/ekf/noise_motion", noise_motion);
    n.getParam("/ekf/noise_measure", noise_measure);

    this->initAll();

    landMark_sub = n.subscribe("/landMarks", 1, &ekf::update, this);
    icpOdom_sub = n.subscribe("/icp_odom", 1, &ekf::predict, this);
	landMark_pub = n.advertise<visualization_msgs::MarkerArray>("/ekf_landMarks",1);
	odom_pub = n.advertise<nav_msgs::Odometry>("/ekf_odom", 1);
}

void ekf::predict(nav_msgs::Odometry odom)
{
	
	//cout<<endl;
	//cout<<"----new frame"<<endl;

	Eigen::MatrixXd Fx = Eigen::MatrixXd::Zero(3,2*this->landMark_num + 3);
	Fx(0,0) = Fx(1,1) = Fx(2,2) = 1;
	
	//decouple trans and rot
	double trans, rot1, rot2;
	double roll, pitch, yaw;
	double x_measure, y_measure, yaw_measure;
	x_measure = odom.pose.pose.position.x;
	y_measure = odom.pose.pose.position.y;
	
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
	yaw_measure = this->angleNorm(yaw);
	
	trans = std::sqrt(std::pow(x_measure-this->robot_x,2) + std::pow(y_measure-this->robot_y,2));
	rot1 = std::atan2(y_measure-this->robot_y, x_measure-this->robot_x) - this->robot_theta;
	rot2 = yaw_measure - std::atan2(y_measure-this->robot_y, x_measure-this->robot_x);

	this->robot_x = x_measure;
	this->robot_y = y_measure;
	this->robot_theta = yaw_measure;

	//estimate mean
	this->estimated_status = this->status;
	this->estimated_status(0) += trans * std::cos(this->status(2)+rot1);
	this->estimated_status(1) += trans * std::sin(this->status(2)+rot1); 
	this->estimated_status(2) += ( rot1 + rot2 );
	this->estimated_status(2) = this->angleNorm(this->estimated_status(2));	


	//estimate covariance
	Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(2*this->landMark_num+3, 2*this->landMark_num+3);
	Gt(0,2) = -trans* std::sin(rot1 + this->status(2));
	Gt(1,2) = trans * std::cos(rot1 + this->status(2));

	this->estimated_covariance = Gt * this->covariance * Gt.transpose() + Fx.transpose() * this->noise_R * Fx;
	
	//cout<<"----prev status "<<status(0)<<" "<<status(1)<<" "<<status(2)<<endl;
	//cout<<"----predict status "<<estimated_status(0)<<" "<<estimated_status(1)<<" "<<estimated_status(2)<<endl;


	this->updateFeatureMap(this->landMark);
	this->publishResult();
	

}

void ekf::update(visualization_msgs::MarkerArray input)
{   

    MatrixXd landMarkFeatures = this->landMarksToXY(input);
   
	this->landMark = landMarkFeatures;
}

void ekf::initAll()
{   


	this->status = Eigen::VectorXd::Zero(3+2*this->landMark_num);
	this->covariance = Eigen::MatrixXd::Zero(2*this->landMark_num+3, 2*this->landMark_num+3);
	for(int i=0; i<2*this->landMark_num; i++)
	{
		this->covariance(i+3, i+3) = 9999;
	}

	this->noise_Q = Eigen::MatrixXd::Identity(2,2);
	this->noise_R = Eigen::MatrixXd::Identity(3,3);
	this->noise_Q = this->noise_measure * this->noise_Q;
	this->noise_R = this->noise_motion * this->noise_R;
	
	this->new_point = 0;
	this->isFirstScan = true;
	//cout<<"----------init successfully"<<endl;
}

Eigen::MatrixXd ekf::landMarksToXY(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

void ekf::updateFeatureMap(Eigen::MatrixXd newFeatures)
{   
	for(int i=0; i<newFeatures.cols(); i++)
	{	
		//if landmark never seen before
		Vector2d z = this->cartesianToPolar(newFeatures(0,i), newFeatures(1,i)); 
		int point_num = this->findNearestMap(z);
		if(point_num == this->new_point )
		{	
			this->new_point += 1;
			this->estimated_status(3+point_num*2) = this->estimated_status(0) + z(0)*std::cos(z(1)+this->estimated_status(2));
			this->estimated_status(4+point_num*2) = this->estimated_status(1) + z(0)*std::sin(z(1)+this->estimated_status(2));
			//cout <<"new point x " << this->estimated_status(3+point_num*2)<< " y "<< this->estimated_status(4+point_num*2)  <<endl;
		}
		

		//calculate delta
		Eigen::Vector2d delta;
		delta(0) = this->estimated_status(2*point_num+3) - this->estimated_status(0);
		delta(1) = this->estimated_status(2*point_num+4) - this->estimated_status(1);
		double q = delta.transpose() * delta;
	
		//calculate z_estimate
		Eigen::Vector2d estimated_z;
		estimated_z(0) = std::sqrt(q);
		estimated_z(1) = this->angleNorm( std::atan2(delta(1),delta(0)) - this->estimated_status(2) );
		
		//calculate F
		Eigen::MatrixXd F = Eigen::MatrixXd::Zero(5,2*this->landMark_num + 3);
		F(0,0) = F(1,1) = F(2,2) = F(3,2*point_num+3) = F(4,2*point_num+4) = 1;
	
		//calculate small jacobian
		Eigen::MatrixXd temp(2, 5);
		temp << -std::sqrt(q)/q*delta(0), -std::sqrt(q)/q*delta(1), 0, std::sqrt(q)/q*delta(0), std::sqrt(q)/q*delta(1),
		  delta(1)/q, -delta(0)/q, -1, -delta(1)/q, delta(0)/q;
	
		Eigen::MatrixXd H;
		H = temp * F;
		
		//calculate kalman gain
		Eigen::MatrixXd K;
		K = this->estimated_covariance * H.transpose() * (H*this->estimated_covariance*H.transpose() + this->noise_Q).inverse();
		
		Eigen::Vector2d diff;
		diff = z - estimated_z;
		if(diff(1) > M_PI)
		{
			diff(1) = z(1) - estimated_z(1) - 2*M_PI;
		}
		if(diff(1) < -M_PI)
		{
			diff(1) = z(1) - estimated_z(1) + 2*M_PI;
		}

		this->estimated_status += K*(diff);
		this->estimated_covariance -= K * H * this->estimated_covariance;
		
		//cout << "z " << z(0) <<" "<< z(1) <<endl;
		//cout << "estimated_z " << estimated_z(0) <<" " << estimated_z(1) <<endl;
		//cout << "diff" << diff(1)  << endl;
		//cout << "----update status " << estimated_status(0) <<" "<< estimated_status(1) << " "<<estimated_status(2) <<endl;
	}


	this->status = this->estimated_status;
	this->status(2) = this->angleNorm( this->estimated_status(2) );
	this->covariance = this->estimated_covariance;

}


int ekf::findNearestMap(Vector2d point)
{  
	int min_point = 0;
	double min_dist = 999, dist;
	Eigen::Vector2d temp_point;

	//transform to global
	Eigen::Vector2d my_point;
	my_point(0) = this->estimated_status(0) + point(0)*std::cos(this->estimated_status(2)+point(1));
	my_point(1) = this->estimated_status(1) + point(0)*std::sin(this->estimated_status(2)+point(1));

	for(int i=0; i<this->new_point; i++)
	{	
		//transform to global corodinate
		temp_point(0) = this->status(3+i*2);
		temp_point(1) = this->status(4+i*2);
		dist = this->calc_dist(my_point, temp_point);
		if(dist < min_dist)
		{
			min_dist = dist;
			min_point = i;
		}
	}
	if(min_dist > this->match_th && this->new_point < this->landMark_num)
	{
		min_point = this->new_point;
		cout<<endl;
		cout <<"----new point num"<<this->new_point<<endl;
		cout <<"----min dist" << min_dist << endl;

	}
	else
	{
		cout<<endl;
		cout <<"----prev point num"<<min_point<<endl;
		cout <<"----min dist" << min_dist <<endl;
	}


		return min_point;
}


Vector2d ekf::cartesianToPolar(double x, double y)
{
    float r = std::sqrt(x*x + y*y);
    float phi = angleNorm(std::atan2(y, x));
    Vector2d r_phi(r, phi);
    return r_phi;
}

float ekf::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{   
    return std::sqrt((pta(0)-ptb(0))*(pta(0)-ptb(0)) + (pta(1)-ptb(1))*(pta(1)-ptb(1)));
}

double ekf::angleNorm(double angle)
{
    // 0 ~ 360
    while(angle > 2*M_PI)
        angle = angle - 2*M_PI;
    while(angle < 0)
        angle = angle + 2*M_PI;
    return angle;
}

void ekf::publishResult()
{
    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(status(2));

    geometry_msgs::TransformStamped ekf_trans;
    ekf_trans.header.stamp = ros::Time::now();
    ekf_trans.header.frame_id = "map";
    ekf_trans.child_frame_id = "ekf_slam";

    ekf_trans.transform.translation.x = status(0);
    ekf_trans.transform.translation.y = status(1);
    ekf_trans.transform.translation.z = 0.0;
    ekf_trans.transform.rotation = odom_quat;

    ekf_broadcaster.sendTransform(ekf_trans);



	visualization_msgs::MarkerArray landMark_array_msg;
	landMark_array_msg.markers.resize(this->landMark_num);

	for(int i=0; i<this->landMark_num; i++)
	{
		landMark_array_msg.markers[i].header.frame_id = "map";
        landMark_array_msg.markers[i].header.stamp = ros::Time(0);
        landMark_array_msg.markers[i].ns = "lm";
        landMark_array_msg.markers[i].id = i;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        landMark_array_msg.markers[i].pose.position.x = this->status(3+2*i);
        landMark_array_msg.markers[i].pose.position.y = this->status(4+2*i);
        landMark_array_msg.markers[i].pose.position.z = 0; // 2D
        landMark_array_msg.markers[i].pose.orientation.x = 0.0;
        landMark_array_msg.markers[i].pose.orientation.y = 0.0;
        landMark_array_msg.markers[i].pose.orientation.z = 0.0;
        landMark_array_msg.markers[i].pose.orientation.w = 1.0;
        landMark_array_msg.markers[i].scale.x = 0.2;
        landMark_array_msg.markers[i].scale.y = 0.2;
        landMark_array_msg.markers[i].scale.z = 0.2;
        landMark_array_msg.markers[i].color.a = 1; // Don't forget to set the alpha!
        landMark_array_msg.markers[i].color.r = 1.0;
        landMark_array_msg.markers[i].color.g = 0.0;
        landMark_array_msg.markers[i].color.b = 0.0;
    }

    landMark_pub.publish(landMark_array_msg);



	nav_msgs::Odometry odom;
	odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "map";

	odom.pose.pose.position.x = status(0);
	odom.pose.pose.position.y = status(1);
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	odom_pub.publish(odom);






}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n;

    ekf ekf_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}
