#ifndef MEASUREMENT_MODEL_INTERFACE
#define MEASUREMENT_MODEL_INTERFACE

#include<Eigen/Core>

class MeasurementModelInterface 
  
  {

   public:
     
     virtual unsigned int getDimension() const = 0;

     virtual ~MeasurementModelInterface(){ };

     virtual Eigen::VectorXd getMeasurementVector(const Eigen::VectorXd& currState)=0;

     virtual Eigen::MatrixXd getObservationMatrix()=0;

     virtual Eigen::MatrixXd getMeasurementNoiseCovariance()=0;

};

#endif
