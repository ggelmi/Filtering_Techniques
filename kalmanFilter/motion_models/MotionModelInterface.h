#ifndef MOTION_MODEL_INTERFACE
#define MOTION_MODEL_INTERFACE

#include<Eigen/Core>

class MotionModelInterface {

  public:
   
   virtual unsigned int getDimension() const = 0;

   virtual ~MotionModelInterface(){ };

   virtual Eigen::VectorXd predictState( const Eigen::VectorXd& currState ) = 0;

   virtual Eigen::MatrixXd getStateTransitionMatrix(const Eigen::VectorXd& currState)=0;
 
   virtual Eigen::MatrixXd getProcessNoiseCovariance()=0;
     
        


};

#endif
