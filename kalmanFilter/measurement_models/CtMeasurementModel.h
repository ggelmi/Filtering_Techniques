#ifndef CT_MEASURUMENT_MODEL_H
#define CT_MEASURUMENT_MODEL_H

#include"MeasurementModelInterface.h"

class CtMeasurementModel: public  MeasurementModelInterface
{

   public:

      CtMeasurementModel (const double& sig, unsigned int& dim);

      unsigned int getDimension() const;

      ~CtMeasurementModel(){ };

      Eigen::VectorXd getMeasurementVector(const Eigen::VectorXd& currState);

      Eigen::MatrixXd getObservationMatrix();

      Eigen::MatrixXd getMeasurementNoiseCovariance();


  private:

      double sigma;

      unsigned int DIM;
};

#endif
