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
const double sigmaPhi = 0.25;
const double sigmaOmega = 0.5;
unsigned int dimen= 5;

CoordinatedTurnModel  ct_model(Delta,sigmaPhi,sigmaOmega,dimen);

std::cout << "The dimension is : " << ct_model.getDimension() << std::endl;

Eigen::VectorXd currState (dimen);

currState << 1,1,0,2.7,0;

Eigen::VectorXd predState =  ct_model.predictState(currState);

std::cout << "The predicted state : \n " << predState << std::endl;


return 0;
}
