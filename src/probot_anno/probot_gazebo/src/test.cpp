#include <iostream>
#include <Eigen/Dense>
#include "velocity_control.cpp"
using namespace std;
using namespace Eigen;

int main(void){
  Matrix4f Mat;
  Mat << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16;
  Vector3f test;
  test = Mat.block<3,1>(0,3);
  test(1) = 123;
  cout << test << endl;
  cout << Mat << endl;
   
}
