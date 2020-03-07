#include <iostream>
#include <Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

const float PI = 3.1415926;

Matrix4f ComputeMat( float theta, float a, float alpha, float d);
vector<Matrix4f> ForwardMat( float theta);

float alpha[6] = {0, PI/2, 0, PI/2, PI/2, PI/2};
float a[6] = {0, 0, 0.225, 0, 0, 0};
float d[6] = {0.284, 0, 0, 0.2289, 0, 0.055};
float thetaInit[6] = {0, PI/2, 0, PI, PI/2, 0};
float targetTheta[6] = {0.927,-0.687,-0.396,0,1.083,0.927};
/*
            {0.322,-0.855,-0.021,0,0.877,0.322},
            {-0.322,-0.636,-0.011,0,0.647,-0.322}};
*/
vector<Matrix4f> ForwardMat(float theta[]){
  vector<Matrix4f> T;
  for(int i = 0; i < 6; i++){
    Matrix4f tempT;
    tempT = ComputeMat(thetaInit[i]+theta[i], a[i], alpha[i], d[i]);
    if(i>0) tempT = T[i-1] * tempT;
    T.push_back(tempT);
  }
  return T;
}

Matrix4f ComputeMat( float theta, float a, float alpha, float d){

  Matrix4f T;
  T << cos(theta), -sin(theta), 0, a,
  sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d,
  sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d,
  0, 0, 0, 1;
  return T;

}



