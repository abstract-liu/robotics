#include "MotionFunction.h"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <complex>


std::vector<std::vector<double>> BackwardAngle(std::vector<float> pos){

	float sa = std::sin(pos[3]/2 + PI/4);
	float ca = std::cos(pos[3]/2 + PI/4);
	float sb = std::sin(pos[4]/2);
	float cb = std::cos(pos[4]/2);
	float sg = std::sin(pos[5]/2);
	float cg = std::cos(pos[5]/2);
	
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





std::vector<Eigen::Matrix4f> ForwardMat(float theta[]){
	std::vector<Eigen::Matrix4f> T;
  	for(int i = 0; i < 6; i++){
		Eigen::Matrix4f tempT;
    tempT = ComputeMat(thetaInit[i]+theta[i], a[i], alpha[i], d[i]);
    if(i>0) tempT = T[i-1] * tempT;
    T.push_back(tempT);
  }
  return T;
}

Eigen::Matrix4f ComputeMat( float theta, float a, float alpha, float d){

	Eigen::Matrix4f T;
  	T << cos(theta), -sin(theta), 0, a,
  sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
  sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d,
  0, 0, 0, 1;
  return T;

}

Eigen::MatrixXf ComputeJacobi(float theta[]){

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

Eigen::MatrixXf ComputeInvJacobi(float theta[]){

	Eigen::MatrixXf invJacobi(6,6);
	Eigen::MatrixXf jacobi(6,6);

	jacobi = ComputeJacobi(theta);
	invJacobi = jacobi.inverse();
	return invJacobi;
}

void ComputeVelTab(std::vector<std::vector<float>>& velocityTab, int rate){

	float theta[6] = {0,0,0,0,0,0};
	Eigen::MatrixXf invJacobi(6,6);
	Eigen::VectorXf decartesVel(6);
	Eigen::VectorXf angleVel(6);

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
			 velocityTab[i][j] = angleVel(j);
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
			velocityTab[i][j] = angleVel(j);
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
			velocityTab[i][j] = angleVel(j);
			theta[j] += angleVel(j)/rate;
		}
	}

}



std::vector<float> ComputeCoef(float theta0, float theta1, float tf, float v0, float v1, float a0, float a1){

	std::vector<float> coefficient(6,0);
	coefficient[0] = theta0;
	coefficient[1] = v0;
	coefficient[2] = a0/2;
	coefficient[3] = (20*theta1 - 20*theta0 - (8*v1+12*v0)*tf - (3*a0-a1)*std::pow(tf,2))/(2*std::pow(tf,3));
	coefficient[4] = (30*theta0 - 30*theta1 +(14*v1+16*v0)*tf +(3*a0-2*a1)*std::pow(tf,2))/(2*std::pow(tf,4));
	coefficient[5] = (12*theta1 - 12*theta0 - (6*v1+6*v0)*tf - (a0-a1)*std::pow(tf,2))/(2*std::pow(tf,5));

	return coefficient;
}

std::vector<float> AngleMoveStep(int rate, double theta0, double theta1, float tf, float v0, float v1, float a0, float a1){

	std::vector<float> velocityTab(rate*tf,0);
	std::vector<float> coef(6,0);
	float t;

	coef = ComputeCoef(theta0, theta1, tf, v0, v1, a0, a1);
	
	for(int i =0; i <rate*tf; i++){
		t = float(i)/float(rate);
		velocityTab[i] = coef[1] + 2*coef[2]*t + 3*coef[3]*std::pow(t,2) + 4*coef[4]*std::pow(t,3) + 5*coef[5]*std::pow(t,4);
	}
//	ROS_INFO_STREAM("ESTIMATEANGLE"<<coef[0]+coef[1]*tf+coef[2]*std::pow(tf,2)+coef[3]*std::pow(tf,3)+coef[4]*std::pow(tf,4)+coef[5]*std::pow(tf,5));
	return velocityTab;
}

//please note that this function will return a vector with length of rate*time+1
std::vector<float> AngleMove(int rate, std::vector<double> theta, std::vector<float> tf, std::vector<float> v, std::vector<float> a){
	
	int length = tf.size();
	std::vector<float> velTable, tempTable;

	for(int i=0; i<length; i++){
		tempTable = AngleMoveStep(rate, theta[i], theta[i+1], tf[i], v[i], v[i+1], a[i], a[i+1]);
		velTable.insert(velTable.end(), tempTable.begin(), tempTable.end());
	}
	velTable.push_back(v[length]);
	return velTable;
}


std::vector<std::vector<float>> GeneratePath( int rate, std::vector<std::vector<float>> vel,  std::vector<std::vector<float>> a, std::vector<float> tf,  std::vector<std::vector<float>> pos){

	int points = vel[0].size();
	std::vector<std::vector<double>> angles(6, std::vector<double>(points,0));
	std::vector<std::vector<double>> tempAngles;
	std::vector<std::vector<float>> velTable;
	std::vector<float> tempVel;
	
	//FBI warning
	for(int j=0;j<6;j++) angles[j][0] = 0;
	for(int i=1; i<points; i++){
		tempAngles = BackwardAngle(pos[i]);
		for(int j=0; j<6;j++){
			angles[j][i] = tempAngles[0][j];
		}	
		ROS_INFO_STREAM("TRUEANGLE1 "<<angles[0][i] - angles[0][i-1]);
		ROS_INFO_STREAM("TRUEANGLE2 "<<angles[1][i] - angles[1][i-1]);
		ROS_INFO_STREAM("TRUEANGLE3 "<<angles[2][i] - angles[2][i-1]);
		ROS_INFO_STREAM("TRUEANGLE4 "<<angles[3][i] - angles[3][i-1]);
		ROS_INFO_STREAM("TRUEANGLE5 "<<angles[4][i] - angles[4][i-1]);
		ROS_INFO_STREAM("TRUEANGLE6 "<<angles[5][i] - angles[5][i-1]);
		
	}

	
	for(int i=0; i<6; i++){
		tempVel = AngleMove(rate, angles[i], tf, vel[i], a[i] );
		velTable.push_back(tempVel);
	}

	return velTable;
}

Eigen::Matrix3f ComputeRot(float alpha, float beta, float gamma){
	Eigen::Matrix3f Rot;
	Rot << std::cos(alpha)*std::cos(beta), std::cos(alpha)*std::sin(beta)*std::sin(gamma) - std::sin(alpha)*std::cos(gamma), std::cos(alpha)*std::sin(beta)*std::cos(gamma) + std::sin(alpha)*std::sin(gamma),
		std::sin(alpha)*std::cos(beta), std::sin(alpha)*std::sin(beta)*std::sin(gamma) + std::cos(alpha)*std::cos(gamma), std::sin(alpha)*std::sin(beta)*std::cos(gamma) - std::cos(alpha)*std::sin(gamma),
		-std::sin(beta), std::cos(beta)*std::sin(gamma), std::cos(beta)*std::cos(gamma);
	return Rot;
}


std::vector<std::vector<float>> FixedRotation(int rate, std::vector<float> pos1, std::vector<float> pos2, int pointsNum){

	Eigen::Matrix3f Rot1;
	Eigen::Matrix3f Rot2;
	Eigen::Matrix3f Rot;
	Rot1 = ComputeRot(pos1[5], pos1[4], pos1[3]);
	Rot2 = ComputeRot(pos2[5], pos2[4], pos2[3]);
	Rot = Rot2 * Rot1.inverse();

	double kx, ky, kz, theta, deltaTheta;
	theta = std::acos(( Rot(0,0)+Rot(2,2)+Rot(1,1)-1)/2 );
	kx = (Rot(2,1)-Rot(1,2))/(2*std::sin(theta));
	ky = (Rot(0,2)-Rot(2,0))/(2*std::sin(theta));
	kz = (Rot(1,0)-Rot(0,1))/(2*std::sin(theta));
	deltaTheta = 0;

	double vtheta, ctheta, stheta;
	double alpha, beta, gamma;

	std::vector<std::vector<float>> pos;
	std::vector<float> tempPose(6,0);
	std::vector<std::vector<float>> vel(6,std::vector<float>(pointsNum+2, 0));
	std::vector<std::vector<float>> a(6, std::vector<float>(pointsNum +2, 0));
	std::vector<float> tf(pointsNum +1, 0);
	
	tempPose[0] = pos1[0];
	tempPose[1] = pos1[1];
	tempPose[2] = pos1[2];

	pos.push_back(pos1);
	for(int i=1; i < pointsNum+1; i++){
		deltaTheta = theta*double(i)/double( pointsNum+1 );
		vtheta = 1 - std::cos(deltaTheta);
		ctheta = std::cos(deltaTheta);
		stheta = std::sin(deltaTheta);
		
		Eigen::Matrix3f tempRot;
		tempRot << kx*kx*vtheta + ctheta, kx*ky*vtheta - kz*stheta, kx*kz*vtheta + ky*stheta,
				kx*ky*vtheta + kz*stheta, ky*ky*vtheta+ctheta, ky*kz*vtheta - kx*stheta,
				kx*kz*vtheta - ky*stheta, ky*kz*vtheta + kx*stheta, kz*kz*vtheta + ctheta;
		tempRot = tempRot * Rot1;
		beta = std::atan2(-tempRot(2,0), std::sqrt(std::pow(tempRot(0,0),2) + std::pow(tempRot(1,0),2) ) );
		alpha = std::atan2(tempRot(1,0)/std::cos(beta), tempRot(0,0)/std::cos(beta));
		gamma = std::atan2(tempRot(2,1)/std::cos(beta), tempRot(2,2)/std::cos(beta));
		ROS_INFO_STREAM("gamma" << gamma  <<"beta" << beta <<"alpha"<<alpha);

		tempPose[3] = gamma;
		tempPose[4] = beta;
		tempPose[5] = alpha;

		pos.push_back(tempPose);
		for(int j=0; j<6; j++){
			vel[j][i] = 0.005;
			a[j][i] = 0;
		}
		tf[i-1] = 1;
	}
	tf[pointsNum] = 1;
	pos.push_back(pos2);

	std::vector<std::vector<float>> velTable;
	velTable = GeneratePath(rate, vel, a, tf, pos);

	return velTable;
}

std::vector<std::vector<float>> LineaerEuler(int rate, float tf, std::vector<float> pos1, std::vector<float> pos2){
	
	std::vector<float> alpha, beta, gamma;
	
	gamma = AngleMoveStep(rate, pos1[3], pos2[3], tf, 0, 0, 0, 0);
	beta = AngleMoveStep(rate, pos1[4], pos2[4], tf, 0, 0, 0, 0);
	alpha = AngleMoveStep(rate, pos1[5], pos2[5], tf, 0, 0, 0, 0);
	gamma.push_back(0);
	beta.push_back(0);
	alpha.push_back(0);


	std::vector<std::vector<float>> velTable(6, std::vector<float>(gamma.size(),0));
	Eigen::MatrixXf invJacobi(6,0);
	Eigen::VectorXf decartesVel(6);
	Eigen::VectorXf angleVel(6);
	
	float theta[6] = {0,0,0,0,0,0};

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
			theta[j] += angleVel(j)/float(rate);
		}
	}
	return velTable;

}

/*
int main(void){
	std::vector<float> test;
	std::vector<double> theta = {10,20};
	std::vector<float> tf = {2};
	std::vector<float> v = {10, 5};
	std::vector<float> a = {0, 3};

    test = AngleMove(10,theta,tf,v,a);

	for(int i =0;i<21;i++){
	std::cout << test[i] << std::endl;
	}
}*/
