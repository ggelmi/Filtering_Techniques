#include<cmath>


#include"ConstantVelocityModel.h"




ConstantVelocityModel::ConstantVelocityModel(const double dtime, 
		                             const double sig, const unsigned int dimension)
 {

	dt = dtime;
	sigma = sig;
	DIM = dimension;
 }
 
const unsigned int ConstantVelocityModel:: getDimension()
   {

     return DIM;
   }

   Eigen::VectorXd ConstantVelocityModel:: predictState( const Eigen::VectorXd currState)
   {

     
     Eigen::VectorXd predicted_state =  getStateTransitionMatrix()*currState;


     return predicted_state; 

   }

Eigen::MatrixXd ConstantVelocityModel:: getStateTransitionMatrix()
   {
	//constexpr unsigned int dim = getDimension();
        Eigen::Matrix<double, getDimension(), getDimension()> stateTransition;

	stateTransition << 1,0,dt,0,
		           0,1,0,dt,
                           0,0,1,0,
			   0,0,0,0;

       return stateTransitionMatrix;	
	                  
   }

Eigen::MatrixXd ConstantVelocityModel::  getProcessNoiseMatrix()
   {
       constexpr unsigned int dim = getDimension();
       Eigen::Matrix<double, dim, dim> processNoise;

       processNoise << std::pow(dt, 4)/4,  0, std::pow(dt, 3)/2,0,
                       0, std::pow(dt, 4)/4, 0, std::pow(dt, 3)/2,
                       std::pow(dt, 3)/2, 0,  std::pow(dt, 2), 0,
                       0,std::pow(dt, 3)/2, 0,    std::pow(dt, 2);

       return processNoise;
   }



