#ifndef _MOTIONFUNCTION_H_
#define _MOTIONFUNCTION_H_

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"

const float PI = 3.1415926;
const float alpha[6] = {0, PI/2, 0, PI/2, PI/2, PI/2};
const float a[6] = {0, 0, 0.225, 0, 0, 0};
const float d[6] = {0.284, 0, 0, 0.2289, 0, 0.055};
const float thetaInit[6] = {0, PI/2, 0, PI, PI/2, 0};
const float targetTheta_1[6] = {0.927,-0.687,-0.396,0,1.083,0.927};
const float targetTheta_2[6] =  {0.322,-0.855,-0.021,0,0.877,0.322};
const float targetTheta_3[6] =  {-0.322,-0.636,-0.011,0,0.647,-0.322};










std::vector<std::vector<double>> BackwardAngle(std::vector<float> pos);

Eigen::Matrix4f ComputeMat( float theta, float a, float alpha, float d);

std::vector<Eigen::Matrix4f> ForwardMat( float theta);

Eigen::Matrix4f ComputeMat(float theta, float a, float alpha, float d);

Eigen::MatrixXf ComputeJacobi(float theta[]);

Eigen::MatrixXf ComputeInvJacobi(float theta[]);

void ComputeVelTab(std::vector<std::vector<float>>& velocityTab, int rate);

std::vector<float> ComputeCoef(float theta0, float theta1, float tf, float v0, float v1, float a0, float a1);

std::vector<float> AngleMoveStep(int rate, double theta0, double theta1, float tf, float v0, float v1, float a0, float a1);


std::vector<float> AngleMove(int rate, std::vector<double> theta, std::vector<float> tf, std::vector<float> v, std::vector<float> a);


//vel[joint][point], a[joint][point]
//pos[point][xyzabg]
std::vector<std::vector<float>> GeneratePath( int rate, std::vector<std::vector<float>> vel,  std::vector<std::vector<float>> a, std::vector<float> tf,  std::vector<std::vector<float>> pos);


Eigen::Matrix3f ComputeRot(float alpha, float beta, float gamma);

std::vector<std::vector<float>> FixedRotation(int rate, std::vector<float> pos1, std::vector<float> pos2, int pointsNum);

std::vector<std::vector<float>> LineaerEuler(int rate, float tf, std::vector<float> pos1, std::vector<float> pos2);
#endif


