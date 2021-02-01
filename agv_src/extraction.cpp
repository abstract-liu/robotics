#include "ros/ros.h"
#include "ros/console.h"
#include <cmath>
#include <ros/duration.h>
#include <stdio.h>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

// structure of the LandMark set, contians the landMarks 
typedef struct{
    std::vector<float> position_x;
    std::vector<float> position_y;
    std::vector<int> id;
} LandMarkSet;


typedef struct{
	double x;
	double y;
} Point;



class extraction{

public:
	extraction(ros::NodeHandle &n);
	~extraction();
    ros::NodeHandle& n;
 
    // get the clusters
    float range_threshold;
    // filter for landmarks extractions
    float radius_max_th;
    // filter for landmarks extractions
    int landMark_min_pt;
	double dist_max_th;

	vector<LandMarkSet> land;

    // listen the ros::laserScan
    ros::Subscriber laser_sub;
    // publish the landMarks as ros::Markers
    ros::Publisher landMark_pub;

    // main process
    void process(sensor_msgs::LaserScan input);
    // filter & extraction process
    LandMarkSet extractLandMark(sensor_msgs::LaserScan input);
	// calculate variance
	void calVariance(const std::vector<Point> &points, Point &center, double &variance, double &distance);

	//fit curve
	bool fitCurve(const std::vector<Point> &points, Point &center, double &radius );

    // publish the landMarks
    void publishLandMark(LandMarkSet input);
    // 2D euclidean distance calculation
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
};

extraction::~extraction()
{}

extraction::extraction(ros::NodeHandle& n):
    n(n)
{   
    // get the params
    n.getParam("/extraction/range_threshold", range_threshold);
    n.getParam("/extraction/radius_max_th", radius_max_th);
    n.getParam("/extraction/landMark_min_pt", landMark_min_pt);
	n.getParam("/extraction/dist_max_th", dist_max_th);
    
    landMark_pub = n.advertise<visualization_msgs::MarkerArray>("landMarks", 1);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &extraction::process, this);
}

void extraction::process(sensor_msgs::LaserScan input)
{   
    double time_0 = (double)ros::Time::now().toSec();

    int label = 0;
    sensor_msgs::LaserScan laser_extr = input;

    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    // init the previous/last point
    Vector2d last_xy(input.ranges[0] * std::cos(input.angle_min), input.ranges[0] * std::sin(input.angle_min));

    // set the intensities as labels
    Vector2d curr_xy;
    float angle, delta_dis;
    for(int i=0; i<total_num; i++)
    {   
        angle = input.angle_min + i * input.angle_increment;
        curr_xy << input.ranges[i] * std::cos(angle), input.ranges[i] * std::sin(angle);
        
        delta_dis = this->calc_dist(curr_xy, last_xy);

        if(delta_dis > range_threshold)
            label++;

        laser_extr.intensities[i] = label;

        last_xy = curr_xy;
    }

    //cout<<"Total original labels: "<<label<<endl;

    LandMarkSet landmarks_ = this->extractLandMark(laser_extr);
	this->publishLandMark(landmarks_);


    double time_1 = (double)ros::Time::now().toSec();
    //cout<<"time_cost:  "<<time_1-time_0<<endl;
}






LandMarkSet extraction::extractLandMark(sensor_msgs::LaserScan input)
{   
    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    LandMarkSet landMarks;

    // TODO: please code by yourself
	float angle ;
	int pre_label = 0, id = 0;
	Point point, center;
	std::vector<Point> points;
	double radius = 0.0, var = 0.0, dist = 0.0;


	for(int i=0; i<total_num; i++){
		
		angle = input.angle_min + i*input.angle_increment;
		
		if(pre_label != input.intensities[i] ){
			
			this->calVariance(points, center, var, dist);
			
			if( var < radius_max_th && points.size() > landMark_min_pt && dist < dist_max_th){ 
				landMarks.id.push_back(id);
				landMarks.position_x.push_back(center.x);
				landMarks.position_y.push_back(center.y);
				id += 1;  
			}
			points.clear();
			pre_label += 1;
			
		}
		
		point.x = input.ranges[i]*std::cos(angle);
		point.y = input.ranges[i]*std::sin(angle);
		points.push_back(point);	
	}


    return landMarks;
}



void extraction::calVariance(const std::vector<Point> &points, Point &center, double &variance, double& distance)
{
	
	double sumX = 0.0, sumY = 0.0;
	double sumDist = 0.0;

	int N = points.size();
	for(int i=0; i<N; i++){
		sumX += points[i].x;
		sumY += points[i].y;
		sumDist += std::sqrt(std::pow(points[i].x,2)+ std::pow(points[i].y,2));
	}

	double centerX = sumX/N, centerY = sumY/N;
	double sumVar = 0.0;
	distance = sumDist /N;

	for(int i=0; i<N; i++){
		sumVar += std::sqrt( std::pow(points[i].x-centerX,2) + std::pow(points[i].y-centerY,2)   );
	}
	
	variance = sumVar /N;
	center.x = centerX;
	center.y = centerY;

}



bool extraction::fitCurve(const std::vector<Point> &points, Point &center, double &radius )
{

	if(points.size() < 3)
		return false;

	double sumX = 0.0, sumY = 0.0;
	double sumX2 = 0.0, sumY2 = 0.0;
	double sumX3 = 0.0, sumY3 = 0.0;
	double sumXY = 0.0, sumXY2 = 0.0, sumX2Y = 0.0;

	int N = points.size();

	for(int i=0; i<N; i++){
	
		double X = points[i].x, Y = points[i].y;

		sumX += X;
		sumY += Y;
		sumX2 += std::pow(X,2);
		sumY2 += std::pow(Y,2);
		sumX3 += std::pow(X,3);
		sumY3 += std::pow(Y,3);
		sumXY += X*Y;
		sumXY2 += X*Y*Y;
		sumX2Y += X*X*Y;
	
	}

	double C,D,E,G,H;
	double a,b,c;

	C = N * sumX2 - sumX * sumX;
	D = N * sumXY - sumX * sumY;
	E = N * sumX3 + N * sumXY2 - (sumX2 + sumY2) * sumX;
	G = N * sumY2 - sumY * sumY;
	H = N * sumX2Y + N * sumY3 - (sumX2 + sumY2) * sumY;
	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D)/ (D * D - G * C);
	c = -(a * sumX + b * sumY +sumX2 + sumY2) / N;

	center.x = a/ (-2);
	center.y = b/ (-2);
	radius = std::sqrt(a * a + b * b - 4 * c)/2;
	
	return true;
}








void extraction::publishLandMark(LandMarkSet input)
{
    if(input.id.size() <= 0)
        return;

    visualization_msgs::MarkerArray landMark_array_msg;

    landMark_array_msg.markers.resize(input.id.size());

    for(int i=0; i<input.id.size(); i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "course_agv__hokuyo__link";
        landMark_array_msg.markers[i].header.stamp = ros::Time(0);
        landMark_array_msg.markers[i].ns = "lm";
        landMark_array_msg.markers[i].id = i;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        landMark_array_msg.markers[i].pose.position.x = input.position_x.at(i);
        landMark_array_msg.markers[i].pose.position.y = input.position_y.at(i);
        landMark_array_msg.markers[i].pose.position.z = 0; // 2D
        landMark_array_msg.markers[i].pose.orientation.x = 0.0;
        landMark_array_msg.markers[i].pose.orientation.y = 0.0;
        landMark_array_msg.markers[i].pose.orientation.z = 0.0;
        landMark_array_msg.markers[i].pose.orientation.w = 1.0;
        landMark_array_msg.markers[i].scale.x = 0.2;
        landMark_array_msg.markers[i].scale.y = 0.2;
        landMark_array_msg.markers[i].scale.z = 0.2;
        landMark_array_msg.markers[i].color.a = 0.3; // Don't forget to set the alpha!
        landMark_array_msg.markers[i].color.r = 0.0;
        landMark_array_msg.markers[i].color.g = 0.0;
        landMark_array_msg.markers[i].color.b = 1.0;
		landMark_array_msg.markers[i].lifetime = ros::Duration();
    }

    landMark_pub.publish(landMark_array_msg);
}

float extraction::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    float dist = 0;
	Eigen::Vector2d diff = pta - ptb;
	dist = diff.norm();
	return dist;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extraction");
    ros::NodeHandle n;

    extraction extraction_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}
