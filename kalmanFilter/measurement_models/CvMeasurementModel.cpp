#include<Eigen/Dense>
#include<cmath>

#include"CvMeasurementModel.h"


 CvMeasurementModel:: CvMeasurementModel (const double& sig, unsigned int& dim)
   {
	sigma = sig;

	DIM = dim;
   }

 unsigned int CvMeasurementModel:: getDimension() const
  {

	return DIM;

  }

 Eigen::VectorXd CvMeasurementModel:: getMeasurementVector(const Eigen::VectorXd& currState)

  {

        Eigen::VectorXd meas_vector = getObservationMatrix()*currState;

	return meas_vector;

  }

 Eigen::MatrixXd CvMeasurementModel:: getObservationMatrix()

  {

	Eigen::MatrixXd observationMatrix (getDimension(),4);


	observationMatrix << 1,0,0,0,
		             0,1,0,0;

	return observationMatrix;

  }

 Eigen::MatrixXd CvMeasurementModel::getMeasurementNoiseCovariance()

  {

	Eigen::MatrixXd measCovariance (getDimension(),getDimension());

        measCovariance << 1,0,
                           0,1;

	measCovariance =  std::pow(sigma,2)*measCovariance;

	return measCovariance;
  }
