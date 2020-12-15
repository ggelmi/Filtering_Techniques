#ifndef CONSTANT_VELOCITY_MODEL_H
#define CONSTANT_VELOCITY_MODEL_H

#include"MotionModelInterface.h"

class ConstantVelocityModel: public  MotionModelInterface
{

  public:

   ConstantVelocityModel(const double& dtime, const double& sig, unsigned int& dimension);

   ~ConstantVelocityModel(){ };

   unsigned int getDimension() const ;
	  
   Eigen::VectorXd predictState( const Eigen::VectorXd& currState);

   Eigen::MatrixXd getStateTransitionMatrix(const Eigen::VectorXd& currState) ;

   Eigen::MatrixXd  getProcessNoiseCovariance() ;


   private:

    double dt;

    double sigma;

    unsigned int DIM;



};

#endif
