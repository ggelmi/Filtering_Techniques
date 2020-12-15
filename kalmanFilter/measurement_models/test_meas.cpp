#include<Eigen/Dense>
#include<iostream>
#include"CvMeasurementModel.h"

#include"CtMeasurementModel.h"


int main()
{

  const double sigma = 0.5;

  unsigned int dimen = 2;

  /**
  CvMeasurementModel cvMeas_model (sigma, dimen);

  std::cout << "The meas dimension: " << cvMeas_model.getDimension() <<std::endl;

  std::cout << "The obs Matrix : \n" << cvMeas_model.getObservationMatrix() << std::endl;

  std::cout << "The cov Matrix: \n" << cvMeas_model.getMeasurementNoiseCovariance() << std::endl;

 
  Eigen::VectorXd currState (4);

  currState << 1,2,0,0;


  Eigen::VectorXd meas =  cvMeas_model.getMeasurementVector(currState);


  std::cout << "The meas : \n " << meas << std::endl;

  **/

  CtMeasurementModel ctMeas_model (sigma, dimen);

  std::cout << "The meas dimension: " << ctMeas_model.getDimension() <<std::endl;

  
  std::cout << "The obs Matrix : \n" << ctMeas_model.getObservationMatrix() << std::endl;

  std::cout << "The cov Matrix: \n" << ctMeas_model.getMeasurementNoiseCovariance() << std::endl;


  Eigen::VectorXd currState (4);

  currState << 1,2,0,0;


  Eigen::VectorXd meas =  ctMeas_model.getMeasurementVector(currState);


  std::cout << "The meas : \n " << meas << std::endl;


}
