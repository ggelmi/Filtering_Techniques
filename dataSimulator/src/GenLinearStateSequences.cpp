
#include<vector>
#include"GenLinearStateSequences.h"
#include<iostream>

using namespace mvnrnd;

namespace dataSimulator
{
    GenLinearStateSequences::GenLinearStateSequences(const Eigen::VectorXd& priorM,const Eigen::MatrixXd& priorC,
                                                     MotionModelInterface*& motionMdl, const unsigned int& N)
        {
            motionModel = motionMdl;
            priorMean = priorM;
            priorCovar = priorC;
            Num=N;
        }
    
    std::vector<Eigen::VectorXd> GenLinearStateSequences:: generateStateSequence()
        {
            auto n = priorMean.rows();
            std::cout << "prior mean is :\n" << priorMean << std::endl;
            std::vector<Eigen::VectorXd> stateSequences(Num + 1);
            // initial sample
            Mvn mvn(priorMean, priorCovar,200);
            Eigen::VectorXd sampleVec= mvn.sample();
            std::cout << "Sample from prior mean dist is :\n" << sampleVec << std::endl;
            stateSequences[0] = sampleVec;

            // Zero mean random noise distribution
            Eigen::VectorXd mean(n);
            mean.setZero();
            Mvn mvnZeroMean(mean,motionModel->getProcessNoiseCovariance(), 200);

            std::cout << "Size of stateSequences: " << stateSequences.size() << std::endl;
            for (unsigned i = 0; i < Num; i++)
            {
                stateSequences[i+1] = motionModel->predictState(stateSequences[i]) 
                                                                            + mvnZeroMean.sample();
                
            }

            return stateSequences;

        }
}