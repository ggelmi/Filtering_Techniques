#ifndef CONSTANT_VELOCITY_MODEL_H
#define CONSTANT_VELOCITY_MODEL_H

#include"MotionModelInterface.h"

class ConstantVelocityModel: public  MotionModelInterface
{

  public:

   ConstantVelocityModel(const double dtime, const double sig, const unsigned int dimension);

   ~ConstantVelocityModel(){ };
   const unsigned int getDimension();
	  
   Eigen::VectorXd predictState( const Eigen::VectorXd currState);

   Eigen::MatrixXd getStateTransitionMatrix();

   Eigen::MatrixXd  getProcessNoiseMatrix();


   private:

    double dt;

    double sigma;

    unsigned int DIM;



};

#endif
