#ifndef DUAL_BEARING_MEAS_MODEL_H
#define DUAL_BEARING_MEAS_MODEL_H

#include"MeasurementModelInterface.h"

class DualBearingMeasModel: public  MeasurementModelInterface
{

   public:

      DualBearingMeasModel (const double& sig, const Eigen::VectorXd& sensorOnePos,const Eigen::VectorXd& sensorTwoPos, unsigned int& dim);

      unsigned int getDimension() const;

      ~DualBearingMeasModel(){ };

      Eigen::VectorXd getMeasurementVector(const Eigen::VectorXd& currState);

      Eigen::MatrixXd getObservationMatrix();

      Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState);

      Eigen::MatrixXd getMeasurementNoiseCovariance();


  private:

      double sigma;

      unsigned int DIM;

      Eigen::VectorXd sensorOnePosition;

      Eigen::VectorXd sensorTwoPosition;
};

#endif
