#include<vector>
#include"GenLinearMeasurementSequences.h"
#include<iostream>

using namespace mvnrnd;

namespace dataSimulator
{
    GenLinearMeasurementSequences::GenLinearMeasurementSequences(MeasurementModelInterface*& measMdl)
        {
            measModel = measMdl;
        }
    
    std::vector<Eigen::VectorXd> GenLinearMeasurementSequences:: generateMeasurementSequence(const std::vector<Eigen::VectorXd>& stateSequences)
        {
            if(stateSequences.size() == 0)
                {
                    std::cerr << "No data in the vector " << std::endl;
                }
            auto n = measModel->getDimension();
            std::cout << "State dimen: " << n << std::endl;
            unsigned int dataSize = stateSequences.size()-1;

            // Zero mean random noise distribution
            Eigen::VectorXd mean(n);
            mean.setZero();
            Mvn mvnZeroMean(mean,measModel->getMeasurementNoiseCovariance(), 200);

            std::vector<Eigen::VectorXd> measurSequences(dataSize);

            for (unsigned i = 0; i < dataSize; i++)
            {
                measurSequences[i] = measModel->getMeasurementVector(stateSequences[i+1]) 
                                                                            + mvnZeroMean.sample();
                
            }
            return measurSequences;

        }
}