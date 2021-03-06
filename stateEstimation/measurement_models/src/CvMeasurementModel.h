/* File CvMeasurementModel.h
*//**
*     @file CvMeasurementModel.h
*     @brief This file declares the constant velocity measurement model
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/14/2020
*
*/

#ifndef CV_MEASURUMENT_MODEL_H
#define CV_MEASURUMENT_MODEL_H

#include"MeasurementModelInterface.h"

namespace stateEstimation
{
    class CvMeasurementModel: public  MeasurementModelInterface
    {
        public:
            CvMeasurementModel (const double& sig, unsigned int& dim);
            
            ~CvMeasurementModel(){ };
            /**
            * @brief Returns the measurement dimension
            *
            * @return unsigned int
            */
            unsigned int getDimension() const;
            /**
            * @brief Get the measurement vector
            *
            * @param currState : Current state.
            * @return Eigen::VectorXd
            */ 
            Eigen::VectorXd getMeasurementVector(const Eigen::VectorXd& currState);
            /**
            * @brief Get the measurement observation matrix
            *
            * @return Eigen::MatrixXd
            */
            Eigen::MatrixXd getObservationMatrix();
            /**
            * @brief Get the measurement noise covariance matrix
            *
            * @return Eigen::MatrixXd
            */
            Eigen::MatrixXd getMeasurementNoiseCovariance();
            /**
            * @brief Get the measurement jacobian matrix
            *
            * @param currState : Current state.
            * @return Eigen::MatrixXd
            */
            Eigen::MatrixXd getJacobianMatrix(const Eigen::VectorXd& currState) const;
        private:
            double sigma;       // standard deviation for the sensor-model noise
            unsigned int DIM;   // Dimension of the measurement space 
    };
}
#endif
