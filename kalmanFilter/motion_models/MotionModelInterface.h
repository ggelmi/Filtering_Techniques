#ifndef MOTION_MODEL_INTERFACE
#define MOTION_MODEL_INTERFACE

#include<Eigen/Core>

class MotionModelInterface {

  public:
   
   virtual const unsigned int getDimension() = 0;

   virtual ~MotionModelInterface(){ };

   virtual Eigen::VectorXd predictState( const Eigen::VectorXd currState ) = 0;

   virtual Eigen::MatrixXd getStateTransitionMatrix() = 0;
 
   virtual Eigen::MatrixXd getProcessNoiseMatrix()=0;
     
        


};

#endif
