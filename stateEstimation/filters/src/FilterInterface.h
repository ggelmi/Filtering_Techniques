/* File FilterInterface.h
*//**
*     @file FilterInterface.h
*     @brief This file declares the  filter interface class
*     
*     @author Guled Elmi, ggredelmi@gmail.com
*     Created 12/20/2020
*
*/

#ifndef FILTER_INTERFACE
#define FILTER_INTERFACE

#include<Eigen/Core>

namespace stateEstimation
{
    class FilterInterface
    {
        public:

            virtual ~FilterInterface(){ }  
            /**
            * @brief Predicts the current state to the next timestamp
            *
            * @param currState : Current state.
            * @return Eigen::VectorXd
            */   
            virtual void predictState( Eigen::VectorXd& state,Eigen::MatrixXd& covMatrix) const = 0; 

            virtual void updateState( Eigen::VectorXd& state, Eigen::MatrixXd& cov, const Eigen::VectorXd& measurement) const = 0;

            //virtual Eigen::VectorXd computeInnovation(const Eigen::VectorXd& predState,const Eigen::VectorXd& measVector) const = 0;

            //virtual Eigen::MatrixXd computeInnovCovariance(const Eigen::MatrixXd& predCov,const Eigen::VectorXd& predState) const = 0;

            //virtual Eigen::MatrixXd computeKalmanGain(const Eigen::MatrixXd& predCov, const Eigen::MatrixXd& innovCovariance) const = 0;
    };
}
#endif