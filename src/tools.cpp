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
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
