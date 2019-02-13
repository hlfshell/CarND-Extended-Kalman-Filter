#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  // Init variables for calculation
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

   //Error check against invalid sizings
  if(estimations.size() <= 0){
     cout << "No estimations - vector was empty!";
     exit(0);
  } else if(estimations.size() != ground_truth.size()){
     cout << "Estimations and ground truth must be the same size!";
     exit(0);
  }

  for(int i = 0; i < estimations.size(); i++){
     VectorXd diff = estimations[i] - ground_truth[i];
     diff = diff.array() * diff.array();
     rmse += diff;
  }

   //Calculate the mean
   rmse = rmse / estimations.size();

   //Calculate squared root
   rmse = rmse.array().sqrt();

   //return resulting rmse
   return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);

   //Recover the state parameters
   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   //Error out if division by zero
   if(px == 0 && py == 0){
      cout << "Error - both px and py are 0, thus creating a divide by zero scenario when calculating the Jacobian";
   }

   //Calculate commonly used sections
   float vxpy = vx * py;
   float vypx = vy * px;
   float px_2 = px * px;
   float py_2 = py * py;
   float denom_2  = px_2 + py_2;
   float denom_sqrt = sqrt(denom_2);
   float vypx_vxpy = vypx - vxpy;
   float denom_32 = pow(denom_sqrt, 3/2);

   Hj << px / denom_sqrt, py / denom_sqrt, 0, 0,
      -1 * py / denom_2, px / denom_2, 0, 0,
      (py * vypx_vxpy)/denom_32, (px*vypx_vxpy)/denom_32, px/denom_sqrt, py/denom_sqrt; 

   return Hj;
}
