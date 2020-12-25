#include<Eigen/Dense>
#include<iostream>
#include<fstream>

#include"Mvn.h"
#include"GenLinearStateSequences.h"
#include"GenLinearMeasurementSequences.h"
#include"KFAdapter.h"
#include"DualBearingMeasModel.h"
#include"CoordinatedTurnModel.h"
#include"EKFAdapter.h"

using namespace dataSimulator;
using namespace mvnrnd;

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

    const double delta = 1;
    const double sig = 0.5;
    unsigned int dim = 4;
    MotionModelInterface* cv_model = new ConstantVelocityModel(delta,sig,dim);
    
    unsigned int num = 10;
    GenLinearStateSequences genTrueState(mean,sigma,cv_model,num);

    std::vector<Eigen::VectorXd> true_data = genTrueState.generateStateSequence();

    std::ofstream output("../../data/groundTruth.mat");
    std::ofstream output_meas("../../data/measurementData.mat");

    // creating measurement model
    const double sigma_meas = 20;
    unsigned int sensor_dim = 2;

    MeasurementModelInterface* meas_model = new CvMeasurementModel(sigma_meas,sensor_dim);

    GenLinearMeasurementSequences genMeasData(meas_model);

    std::vector<Eigen::VectorXd> sensor_data = genMeasData.generateMeasurementSequence(true_data);

    std::cout << "The simulated sensor data size: " << sensor_data.size() << std::endl;

    for(unsigned int i=0; i<num; i++)
    {
        Eigen::VectorXd state = true_data[i];
        output << state(0) << "   "<< state(1)<< "   "<< state(2)<< "   "<< state(3) << std::endl;
    }
    
     for(unsigned int i=0; i<sensor_data.size(); i++)
    {
        Eigen::VectorXd meas = sensor_data[i];
        output_meas << meas(0) << "   "<< meas(1)<< std::endl;
    }

    bool useMatlabData = true;
    std::vector<Eigen::VectorXd> measData;

    if(useMatlabData)
    {
        std::ifstream input;
        //input.open("/home/guuto/octave/Filtering_Techniques/matlab_simulator/linear_simulator/matlab_data/measData.mat");
        input.open("/home/guuto/octave/Filtering_Techniques/matlab_simulator/non_linear_simulator/matlab_data/measData.mat");
        int counter  = 0;
        double x,y;
        std::string line;
	    int i = 0;
	    //std::cout << "Inside useMatlabData :" << std::endl;

        while (std::getline(input,line))
        {
            //std::cout << "inside while " << std::endl;
	        if(counter > 4 && line != "")
            {
		        //std::cout << "inside counter " << std::endl;
                std::istringstream iss(line);
		        iss >> x >> y;
	    	    //std::cout << line << std::endl;
	            //std::cout << "X = " << x << " "<< "Y= " << y << std::endl;
		        Eigen::VectorXd meas_vector(2);
                meas_vector << x,y;
		        measData.push_back(meas_vector);
		        //std:: cout << "measData : \n " << measData[i] << std::endl;
		        i++;
	        }
	        counter++;
	    }

        num = measData.size();
        std::cout << "measData NUM: " << num << std::endl;

    }
    //  utilizing the filter to create the state estimates when only having access to
    //  measurements

    std::ofstream output_estimates("/home/guuto/octave/Filtering_Techniques/matlab_simulator/non_linear_simulator/matlab_data/stateEstimates.mat");
    /**
    FilterInterface* kf_filter = new KFAdapter(cv_model,meas_model);
    Eigen::VectorXd measurement;
    for(unsigned int i=0; i<num; i++)
    {
        kf_filter->predictState(mean,sigma);
        if(useMatlabData)
        {
            measurement = measData[i];
        }else
        {
            measurement = sensor_data[i];
        }
        
        kf_filter->updateState(mean,sigma,measurement);

        output_estimates << mean(0) << "      "<< mean(1) << std::endl;

    }
    **/
    // Running the EKF implementation

    const double Delta = 1;
    const double sigmaV = 1;
    const double sigmaOmega = M_PI/180;
    unsigned int dimen = 5;

    MotionModelInterface* ct_model = new CoordinatedTurnModel(Delta,sigmaV,sigmaOmega,dimen);
    std::cout << "Q : \n" << ct_model->getProcessNoiseCovariance() << std::endl;
    Eigen::VectorXd sensor1(2);
    Eigen::VectorXd sensor2(2);

    const double sigm = 0.5*M_PI/180;
    sensor1 << -200,100;
    sensor2 << -200,-100;

    unsigned int sensorDimen = 2;
    MeasurementModelInterface* dbm_model = new DualBearingMeasModel(sigm,sensor1,sensor2,sensorDimen);
    std::cout << "R : \n" << dbm_model->getMeasurementNoiseCovariance() << std::endl;
    Eigen::VectorXd X(5);
    X << 0, 0,14,0,0;
    Eigen::MatrixXd P(dimen, dimen);
    P << 10,0,0,0,0,
        0,10,0,0,0,
        0,0,2,0,0,
        0,0,0,(M_PI/180),0,
        0,0,0,0,((5*M_PI)/180);
    P = P*P;

    FilterInterface* ekf_filter = new EKFAdapter(ct_model,dbm_model);

    Eigen::VectorXd measurement;
    for(unsigned int i=0; i<num; i++)
    {
        ekf_filter->predictState(X,P);
        if(useMatlabData)
        {
            measurement = measData[i];
        }else
        {
            measurement = sensor_data[i];
        }
        
        ekf_filter->updateState(X,P,measurement);

        output_estimates << X(0) << "      "<< X(1) << std::endl;

    }

}