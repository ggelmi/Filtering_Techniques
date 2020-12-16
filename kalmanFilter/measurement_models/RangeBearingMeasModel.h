#ifndef RANGE_BEARING_MEAS_MODEL_H
#define RANGE_BEARING_MEAS_MODEL_H

#include"MeasurementModelInterface.h"

class RangeBearingMeasModel: public  MeasurementModelInterface
{

   public:

      RangeBearingMeasModel (const double& sig_r,const double& sig_b, const Eigen::VectorXd& sensorPos, unsigned int& dim);

      unsigned int getDimension() const;

      ~RangeBearingMeasModel(){ };

      Eigen::VectorXd getMeasurementVector(const Eigen::VectorXd& currState);

      Eigen::MatrixXd getObservationMatrix();

      Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState);

      Eigen::MatrixXd getMeasurementNoiseCovariance();


  private:

      double sigma_r;

      double sigma_b;

      unsigned int DIM;

      Eigen::VectorXd sensorPosition;
};

#endif

