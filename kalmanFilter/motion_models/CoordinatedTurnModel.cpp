#include<cmath>

#include"CoordinatedTurnModel.h"

CoordinatedTurnModel::CoordinatedTurnModel(const double& dtime,
                                             const double& sigPhi,const double& sigOmega, unsigned int& dimension)
 {

        dt = dtime;
	sigmaPhi= sigPhi;
        sigmaOmega = sigOmega;
        DIM = dimension;
 }

unsigned int CoordinatedTurnModel:: getDimension() const
 {

     return DIM;
 }

Eigen::VectorXd CoordinatedTurnModel:: predictState( const Eigen::VectorXd& currState)
 {

     Eigen::VectorXd x(getDimension());
     
     x << dt*currState(3)*cos(currState(4)), 
          dt*currState(3)*sin(currState(4)),
          0,
	  dt*currState(5),
	  0;

     Eigen::VectorXd predicted_state =  currState + x;


     return predicted_state;

 }

Eigen::MatrixXd CoordinatedTurnModel:: getStateTransitionMatrix(const Eigen::VectorXd& currState)
   {

        Eigen::MatrixXd stateTransitionMatrix (getDimension(),getDimension());

        stateTransitionMatrix << 1,0,dt*cos(currState(4)),-dt*currState(4)*sin(currState(4)),0,
                                 0,1,dt*sin(currState(4)),dt*currState(3)*cos(currState(4)),0,
                                 0,0,1,0,0,
                                 0,0,0,1,dt,
	                         0,0,0,0,1;

       return stateTransitionMatrix;

   }

