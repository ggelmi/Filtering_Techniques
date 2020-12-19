#include<Eigen/Dense>
#include<iostream>
#include"Mvn.h"
#include"GenLinearStateSequences.h"
#include"/home/guuto/octave/Filtering_Techniques/stateEstimation/motion_models/MotionModelInterface.h"
#include"/home/guuto/octave/Filtering_Techniques/stateEstimation/motion_models/ConstantVelocityModel.h"

using namespace dataSimulator;
using namespace stateEstimation;

int main()
{

    // Define the covariance matrix and the mean
    Eigen::MatrixXd sigma(4, 4);
    sigma <<   1, 0,0,0,
                0, 1,0,0,
                0,0,1,0,
                0,0,0,1;

    Eigen::VectorXd mean(4);
    mean << 0, 0,0,0;

    Mvn mvn(mean, sigma,100);
    
    Eigen::VectorXd test(4);
    test << 0, 0,0,0;
    
    std::cout << "pdf : \n" << mvn.pdf(test) << std::endl;
    
    test << -0.6, -0.6,-0.6,-0.6;

    std::cout << "pdf : \n" << mvn.pdf(test) << std::endl;

    std::cout << "sample from dist : \n" << mvn.sample() << std::endl;
    // creating the model
    ConstantVelocityMotionModel cv_model(1,0.5,4);

    //GenLinearStateSequences genTrueState(mean,sigma,)
    /**
    // Define the covariance matrix and the mean

    Eigen::MatrixXd sigma2(2, 2);
    sigma2 << 10, 7,
            7, 5;
    Eigen::VectorXd mean2(2);
    mean2 << 2, 2;
    Mvn mvn2(mean2, sigma2,200);

    // Sample a number of points
    const unsigned int points = 1000;
    Eigen::MatrixXd x(2, points);
    Eigen::VectorXd vector(2);
    for (unsigned i = 0; i < points; i++)
        {
            vector = mvn2.sample();
            x(0, i) = vector(0);
            x(1, i) = vector(1);
        }
    
    // Calculate the mean and convariance of the produces sampled points
    Eigen::VectorXd approx_mean(2);
    Eigen::MatrixXd approx_sigma(2, 2);
    approx_mean.setZero();
    approx_sigma.setZero();

    for (unsigned int i = 0; i < points; i++)
    {
    approx_mean  = approx_mean  + x.col(i);
    approx_sigma = approx_sigma + x.col(i) * x.col(i).transpose();
    }

    approx_mean  = approx_mean  / static_cast<double>(points);
    approx_sigma = approx_sigma / static_cast<double>(points);
    approx_sigma = approx_sigma - approx_mean * approx_mean.transpose();

    std::cout << "Aproximated mean : \n" << approx_mean << std::endl;
    std::cout << "Aproximated cov  : \n" << approx_sigma << std::endl;
    **/

    //GenLinearStateSequences genTrueState()
    
}