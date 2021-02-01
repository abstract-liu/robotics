#include "MotionFunction.h"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <complex>

std::vector<std::vector<double>> BackwardAngle(std::vector<double> pos){

	double sa = std::sin(pos[3]/2 + PI/4);
	double ca = std::cos(pos[3]/2 + PI/4);
	double sb = std::sin(pos[4]/2);
	double cb = std::cos(pos[4]/2);
	double sg = std::sin(pos[5]/2);
	double cg = std::cos(pos[5]/2);
	
	ikfast_kinematics_plugin::IKFastKinematicsPlugin ik;
	bool ret = ik.IKFastKinematicsPlugin::initialize("robot_description","manipulator","base_link","link_6",0.001);
	geometry_msgs::Pose targetPose;
	
	targetPose.position.x = pos[0];
	targetPose.position.y = pos[1];
	targetPose.position.z = pos[2] - 0.022;
	targetPose.orientation.w = ca*cb*cg + sa*sb*sg;
	targetPose.orientation.x = sa*cb*cg - ca*sb*sg;
	targetPose.orientation.y = ca*sb*cg + sa*cb*sg;
	targetPose.orientation.z = ca*cb*sg - sa*sb*cg;

	std::vector<geometry_msgs::Pose> pose;
	pose.push_back(targetPose);

	std::vector<double> seed;
	seed.push_back(0.0);
	seed.push_back(0.0);
	seed.push_back(0.0);
	seed.push_back(0.0);
	seed.push_back(0.0);
	seed.push_back(0.0);

	std::vector<std::vector<double>> res;
	kinematics::KinematicsResult kinematicRes;

	ret = ik.getPositionIK(pose, seed, res, kinematicRes,kinematics::KinematicsQueryOptions());

	return res;
}





std::vector<Eigen::Matrix4f> ForwardMat(double theta[]){
	std::vector<Eigen::Matrix4f> T;
  	for(int i = 0; i < 6; i++){
		Eigen::Matrix4f tempT;
    	tempT = ComputeMat(thetaInit[i]+theta[i], a[i], alpha[i], d[i]);
    	if(i>0) tempT = T[i-1] * tempT;
    	T.push_back(tempT);
	}
  return T;
}

Eigen::Matrix4f ComputeMat( double theta, double a, double alpha, double d){

	Eigen::Matrix4f T;
  	T << cos(theta), -sin(theta), 0, a,
  sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
  sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d,
  0, 0, 0, 1;
  return T;

}

Eigen::MatrixXf ComputeJacobi(double theta[]){

	Eigen::MatrixXf jacobi(6,6);
	std::vector<Eigen::Vector3f> P;
	std::vector<Eigen::Vector3f> Z;
	std::vector<Eigen::Matrix4f> T;

  	T = ForwardMat(theta);

  for(int i =0; i < 6; i++){
    P.push_back(T[i].block<3,1>(0,3));
    Z.push_back(T[i].block<3,1>(0,2));
  }

  for(int i =0; i<6; i++){
   jacobi.block<3,1>(0,i) = Z[i].cross(P[5]-P[i]);
   jacobi.block<3,1>(3,i) = Z[i]; 
  }

  return jacobi;
}

Eigen::MatrixXf ComputeInvJacobi(double theta[]){

	Eigen::MatrixXf invJacobi(6,6);
	Eigen::MatrixXf jacobi(6,6);

	jacobi = ComputeJacobi(theta);
	invJacobi = jacobi.inverse();
	return invJacobi;
}


std::vector<double> ComputeEulerAngle(Eigen::MatrixXf tempRot){

	std::vector<double> eulerAngle(3,0);
	double beta, alpha, gamma;

	beta = std::atan2(-tempRot(2,0), std::sqrt(std::pow(tempRot(0,0),2) + std::pow(tempRot(1,0),2) ) );
	alpha = std::atan2(tempRot(1,0), tempRot(0,0));
	gamma = std::atan2(tempRot(2,1), tempRot(2,2)) ;
	
	eulerAngle[0] = gamma - PI/2;
	eulerAngle[1] = beta;
	eulerAngle[2] = alpha;

	return eulerAngle;

}


std::vector<double> ComputeCoef(double theta0, double theta1, double tf, double v0, double v1, double a0, double a1){

	std::vector<double> coefficient(6,0);
	coefficient[0] = theta0;
	coefficient[1] = v0;
	coefficient[2] = a0/2;
	coefficient[3] = (20*theta1 - 20*theta0 - (8*v1+12*v0)*tf - (3*a0-a1)*std::pow(tf,2))/(2*std::pow(tf,3));
	coefficient[4] = (30*theta0 - 30*theta1 +(14*v1+16*v0)*tf +(3*a0-2*a1)*std::pow(tf,2))/(2*std::pow(tf,4));
	coefficient[5] = (12*theta1 - 12*theta0 - (6*v1+6*v0)*tf - (a0-a1)*std::pow(tf,2))/(2*std::pow(tf,5));

	return coefficient;
}



























std::vector<std::vector<double>> ComputeExp2Vel(int rate){

	double theta[6] = {0,0,0,0,0,0};
	Eigen::MatrixXf invJacobi(6,6);
	Eigen::VectorXf decartesVel(6);
	Eigen::VectorXf angleVel(6);
	std::vector<std::vector<double>> velocityTab(6, std::vector<double>(12*rate, 0));

	decartesVel(1) = 0;
	decartesVel(3) = 0;
	decartesVel(4) = 0;
	decartesVel(5) = 0; 

	//first stage
	for(int i=0; i<rate*4; i++){
		decartesVel(0) = std::sqrt(2)/4*0.01*i/rate;
		decartesVel(2) = -std::sqrt(2)/4*0.01*i/rate;

		invJacobi = ComputeInvJacobi(theta);
		angleVel = invJacobi*decartesVel;
		
		for(int j=0; j<6; j++){
			 velocityTab[j][i] = angleVel(j);
			 theta[j] += angleVel(j)/rate;
		}
	}

	//second stage
	for(int i=rate*4; i<rate*7; i++){
		decartesVel(0) = std::sqrt(2)*0.01;
		decartesVel(2) = -std::sqrt(2)*0.01;

		invJacobi = ComputeInvJacobi(theta);
		angleVel = invJacobi*decartesVel;
		
		for(int j=0;j<6;j++){
			velocityTab[j][i] = angleVel(j);
			theta[j] += angleVel(j)/rate;
		}
	}

	//third stage
	for(int i=rate*7; i<rate*12; i++){
		decartesVel(0) = std::sqrt(2)*2.4*0.01 - std::sqrt(2)/5*0.01*i/rate;
		decartesVel(2) = -std::sqrt(2)*2.4*0.01 + std::sqrt(2)/5*0.01*i/rate;

		invJacobi = ComputeInvJacobi(theta);
		angleVel = invJacobi*decartesVel;

		for(int j=0; j<6; j++){
			velocityTab[j][i] = angleVel(j);
			theta[j] += angleVel(j)/rate;
		}
	}
	
	return velocityTab;
}





















std::vector<double> QuinticInterpolation(int rate, double theta0, double theta1, double tf, double v0, double v1, double a0, double a1){

	std::vector<double> velocityTab(rate*tf,0);
	std::vector<double> coef(6,0);
	double t;

	coef = ComputeCoef(theta0, theta1, tf, v0, v1, a0, a1);
	
	for(int i =0; i <rate*tf; i++){
		t = double(i)/double(rate);
		velocityTab[i] = coef[1] + 2*coef[2]*t + 3*coef[3]*std::pow(t,2) + 4*coef[4]*std::pow(t,3) + 5*coef[5]*std::pow(t,4);
	}
	return velocityTab;
}

//please note that this function will return a vector with length of rate*time+1
std::vector<double> Interpolation(int rate, std::vector<double> theta, std::vector<double> tf, std::vector<double> v, std::vector<double> a){
	
	int length = tf.size();
	std::vector<double> velTable, tempTable;

	for(int i=0; i<length; i++){
		tempTable = QuinticInterpolation(rate, theta[i], theta[i+1], tf[i], v[i], v[i+1], a[i], a[i+1]);
		velTable.insert(velTable.end(), tempTable.begin(), tempTable.end());
	}
	velTable.push_back(v[length]);
	return velTable;
}


std::vector<std::vector<double>> GeneratePath( int rate, std::vector<std::vector<double>> vel,  std::vector<std::vector<double>> a, std::vector<double> tf,  std::vector<std::vector<double>> pos){

	int points = vel[0].size();
	std::vector<std::vector<double>> angles(6, std::vector<double>(points,0));
	std::vector<std::vector<double>> tempAngles;
	std::vector<std::vector<double>> velTable;
	std::vector<double> tempVel;
	
	//FBI warning
	for(int j=0;j<6;j++) angles[j][0] = 0;
	for(int i=1; i<points; i++){
		tempAngles = BackwardAngle(pos[i]);
		for(int j=0; j<6;j++){
			angles[j][i] = tempAngles[0][j];
		}	
		ROS_INFO_STREAM("TRUEANGLE1 "<<angles[0][i]);
		ROS_INFO_STREAM("TRUEANGLE2 "<<angles[1][i]);
		ROS_INFO_STREAM("TRUEANGLE3 "<<angles[2][i]);
		ROS_INFO_STREAM("TRUEANGLE4 "<<angles[3][i]);
		ROS_INFO_STREAM("TRUEANGLE5 "<<angles[4][i]);
		ROS_INFO_STREAM("TRUEANGLE6 "<<angles[5][i]);
		
	}

	
	for(int i=0; i<6; i++){
		tempVel = Interpolation(rate, angles[i], tf, vel[i], a[i] );
		velTable.push_back(tempVel);
	}

	return velTable;
}





















std::vector<std::vector<double>> EulerInterp(int rate, double tf, std::vector<double> pos1, std::vector<double> pos2){
	
	std::vector<double> alpha, beta, gamma;
	
	gamma = QuinticInterpolation(rate, pos1[3], pos2[3], tf, 0, 0, 0, 0);
	beta =  QuinticInterpolation(rate, pos1[4], pos2[4], tf, 0, 0, 0, 0);
	alpha = QuinticInterpolation(rate, pos1[5], pos2[5], tf, 0, 0, 0, 0);
	gamma.push_back(0);
	beta.push_back(0);
	alpha.push_back(0);


	std::vector<std::vector<double>> velTable(6, std::vector<double>(gamma.size(),0));
	Eigen::MatrixXf invJacobi(6,0);
	Eigen::VectorXf decartesVel(6);
	Eigen::VectorXf angleVel(6);
	
	double theta[6] = {0,0,0,0,0,0};

	decartesVel(0) = 0;
	decartesVel(1) = 0;
	decartesVel(2) = 0;


	for(int i=0; i<gamma.size(); i++){
		decartesVel(3) = gamma[i];
		decartesVel(4) = beta[i];
		decartesVel(5) = alpha[i];

		invJacobi = ComputeInvJacobi(theta);
		angleVel = invJacobi * decartesVel;

		for(int j=0; j<6; j++){
			velTable[j][i] = angleVel(j);
			theta[j] += angleVel(j)/double(rate);
		}
	}
	return velTable;

}




std::vector<std::vector<double>>  HitBell ( int rate, double theta[], std::vector<std::vector<double>> vel,  std::vector<std::vector<double>> a, std::vector<double> tf,  std::vector<std::vector<double>> pos){

	std::vector<double> xVel, yVel, zVel;
	
	xVel = Interpolation(rate, pos[0], tf, vel[0], a[0]);
	yVel = Interpolation(rate, pos[1], tf, vel[1], a[1]);
	zVel = Interpolation(rate, pos[2], tf, vel[2], a[2]);

	Eigen::MatrixXf invJacobi(6,0);
	Eigen::VectorXf decartesVel(6);
	Eigen::VectorXf angleVel(6);
	

	decartesVel(3) = 0;
	decartesVel(4) = 0;
	decartesVel(5) = 0;
	
	std::vector<std::vector<double>> velTable(6, std::vector<double>(xVel.size(), 0));

	for(int i=0; i<xVel.size(); i++){
		decartesVel(0) = xVel[i];
		decartesVel(1) = yVel[i];
		decartesVel(2) = zVel[i];

		invJacobi = ComputeInvJacobi(theta);
		angleVel = invJacobi * decartesVel;

		for(int j=0; j<6; j++){
			velTable[j][i] = angleVel(j);
			theta[j] += angleVel(j)/double(rate);

			if(angleVel(j) > 0.7 && j!= 4) ROS_INFO_STREAM("JOINT"<<j<<"OVERSPEED");


		}
	}
		

	return velTable;

}
