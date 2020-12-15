#include<Eigen/Dense>
#include<iostream>
#include"ConstantVelocityModel.h"
#include"CoordinatedTurnModel.h"

int main()
{
/**
const double delta = 1;
const double sigma = 0.5;
unsigned int dim = 4;

ConstantVelocityModel  cv_model(delta,sigma,dim);

std::cout << "The dimension is : " << cv_model.getDimension() << std::endl;

std::cout << "The stateTransition : \n " << cv_model.getStateTransitionMatrix() << std::endl;
  
std::cout << "The processNoise : \n " << cv_model.getProcessNoiseMatrix() << std::endl;

Eigen::VectorXd currState (dim);

currState << 1,1,0,0;

Eigen::VectorXd predState =  cv_model.predictState(currState);

std::cout << "The predicted state : \n " << predState << std::endl;

**/
// testing the coordinated turn model

const double Delta = 1;
const double sigmaV = 0.01;
const double sigmaOmega = 0.2;
unsigned int dimen= 5;

CoordinatedTurnModel  ct_model(Delta,sigmaV,sigmaOmega,dimen);

std::cout << "The dimension is : " << ct_model.getDimension() << std::endl;

Eigen::VectorXd currState (dimen);

currState << 1,1,0.3,2.7,0;

std::cout << "currState is " << currState << std::endl;

Eigen::VectorXd predState =  ct_model.predictState(currState);

std::cout << "The predicted state : \n " << predState << std::endl;

std::cout << "The processNoise : \n" << ct_model.getProcessNoiseCovariance() << std::endl;

std::cout << "The jacobian : \n" << ct_model.getJacobianMatrix(currState) << std::endl;

return 0;
}
