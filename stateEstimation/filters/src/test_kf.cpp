#include<Eigen/Dense>
#include<iostream>

#include"KFAdapter.h"
#include"CvMeasurementModel.h"
#include"ConstantVelocityModel.h"
#include<iostream>

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

    //creating the motion model
    const double delta = 1;
    const double sig = 0.5;
    unsigned int dim = 4;
    MotionModelInterface* cv_model = new ConstantVelocityModel(delta,sig,dim);

    // creating measurement model
    const double sigma_meas = 20;
    unsigned int sensor_dim = 2;

    MeasurementModelInterface* meas_model = new CvMeasurementModel(sigma_meas,sensor_dim);

    // creating the kf filter

    FilterInterface* kf_filter = new KFAdapter(cv_model,meas_model);

    std::cout << "mean before prediction: \n" << mean << std::endl;
    std::cout << "cov before prediction: \n"<< sigma << std::endl;

    kf_filter->predictState(mean,sigma);

    std::cout << "mean after prediction: \n" << mean << std::endl;
    std::cout << "cov mean prediction: \n"<< sigma << std::endl;

    Eigen::VectorXd measVector(2);
    measVector << 5,5;

    kf_filter->updateState(mean,sigma,measVector);

    std::cout << "mean after update: \n" << mean << std::endl;
    std::cout << "cov mean update: \n"<< sigma << std::endl;

    return 0;
}