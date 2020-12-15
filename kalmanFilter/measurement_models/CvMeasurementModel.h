#ifndef CV_MEASURUMENT_MODEL_H
#define CV_MEASURUMENT_MODEL_H

#include"MeasurementModelInterface.h"

class CvMeasurementModel: public  MeasurementModelInterface
{

   public:

      CvMeasurementModel (const double& sig, unsigned int& dim);

      unsigned int getDimension() const;

      ~CvMeasurementModel(){ };

      Eigen::VectorXd getMeasurementVector(const Eigen::VectorXd& currState);

      Eigen::MatrixXd getObservationMatrix();

      Eigen::MatrixXd getMeasurementNoiseCovariance();


  private:

      double sigma;

      unsigned int DIM;
};

#endif
