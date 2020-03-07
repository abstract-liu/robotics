#include <iostream>
#include <Eigen/Dense>
#include "forward_kinematics.cpp"
#include <vector>

using namespace std;
using namespace Eigen;

MatrixXf ComputeJacobi(float theta[]){

  MatrixXf jacobi(6,6);
  vector<Vector3f> P;
  vector<Vector3f> Z;
  vector<Matrix4f> T;

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

