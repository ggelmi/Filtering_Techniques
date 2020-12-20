#include<Eigen/Dense>
#include<iostream>
#include"Mvn.h"
#include"GenLinearStateSequences.h"
#include"MotionModelInterface.h"
#include"ConstantVelocityModel.h"

using namespace dataSimulator;
using namespace mvnrnd;
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

    Mvn mvn1(mean, sigma,100);
    
    Eigen::VectorXd test(4);
    test << 0, 0,0,0;
    
    std::cout << "pdf : \n" << mvn1.pdf(test) << std::endl;
    
    test << -0.6, -0.6,-0.6,-0.6;

    std::cout << "pdf : \n" << mvn1.pdf(test) << std::endl;

    std::cout << "sample from dist : \n" << mvn1.sample() << std::endl;
    // creating the model
    const double delta = 1;
    const double sig = 0.5;
    unsigned int dim = 4;
    MotionModelInterface* cv_model = new ConstantVelocityModel(delta,sig,dim);
    
    unsigned int num = 10;
    GenLinearStateSequences genTrueState(mean,sigma,cv_model,num);

    std::vector<Eigen::VectorXd> sensor_data = genTrueState.generateStateSequence();

    //std::cout << "The number of generated states are: " << sensor_data.size() << std::endl;
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