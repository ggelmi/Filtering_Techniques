#ifndef BEARING_MEASURUMENT_MODEL_H
#define BEARING_MEASURUMENT_MODEL_H

#include"MeasurementModelInterface.h"

class BearingMeasurementModel: public  MeasurementModelInterface
{

   public:

      BearingMeasurementModel (const double& sig, const Eigen::VectorXd& sensorPos, unsigned int& dim);

      unsigned int getDimension() const;

      ~BearingMeasurementModel(){ };

      Eigen::VectorXd getMeasurementVector(const Eigen::VectorXd& currState);

      Eigen::MatrixXd getObservationMatrix();

      Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState);

      Eigen::MatrixXd getMeasurementNoiseCovariance();


  private:

      double sigma;

      unsigned int DIM;

      Eigen::VectorXd sensorPosition;
};

#endif

