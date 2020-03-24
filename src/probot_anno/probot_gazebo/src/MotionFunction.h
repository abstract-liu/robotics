#ifndef _MOTIONFUNCTION_H_
#define _MOTIONFUNCTION_H_

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"

const double PI = 3.1415926;
const double alpha[6] = {0, PI/2, 0, PI/2, PI/2, PI/2};
const double a[6] = {0, 0, 0.225, 0, 0, 0};
const double d[6] = {0.284, 0, 0, 0.2289, 0, 0.055};
const double thetaInit[6] = {0, PI/2, 0, PI, PI/2, 0};
const double targetTheta_1[6] = {0.927,-0.687,-0.396,0,1.083,0.927};
const double targetTheta_2[6] =  {0.322,-0.855,-0.021,0,0.877,0.322};
const double targetTheta_3[6] =  {-0.322,-0.636,-0.011,0,0.647,-0.322};









std::vector<double> ComputeEulerAngle(Eigen::MatrixXf tempRot);

std::vector<std::vector<double>> BackwardAngle(std::vector<double> pos);

Eigen::Matrix4f ComputeMat( double theta, double a, double alpha, double d);

std::vector<Eigen::Matrix4f> ForwardMat( double theta);

Eigen::Matrix4f ComputeMat(double theta, double a, double alpha, double d);

Eigen::MatrixXf ComputeJacobi(double theta[]);

Eigen::MatrixXf ComputeInvJacobi(double theta[]);

std::vector<std::vector<double>> ComputeExp2Vel(int rate);

std::vector<double> ComputeCoef(double theta0, double theta1, double tf, double v0, double v1, double a0, double a1);

std::vector<double> QuinticInterpolation(int rate, double theta0, double theta1, double tf, double v0, double v1, double a0, double a1);


std::vector<double> Interpolation(int rate, std::vector<double> theta, std::vector<double> tf, std::vector<double> v, std::vector<double> a);


//vel[joint][point], a[joint][point]
//pos[point][xyzabg]
std::vector<std::vector<double>> GeneratePath( int rate, std::vector<std::vector<double>> vel,  std::vector<std::vector<double>> a, std::vector<double> tf,  std::vector<std::vector<double>> pos);

std::vector<std::vector<double>> EulerInterp(int rate, double tf, std::vector<double> pos1, std::vector<double> pos2);



std::vector<std::vector<double>>  HitBell ( int rate, double theta[],  std::vector<std::vector<double>> vel,  std::vector<std::vector<double>> a, std::vector<double> tf,  std::vector<std::vector<double>> pos);



#endif


