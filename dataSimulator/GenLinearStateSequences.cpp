
#include<vector>
#include"GenLinearStateSequences.h"

namespace dataSimulator
{
    GenLinearStateSequences::GenLinearStateSequences(const Eigen::VectorXd& priorM,const Eigen::MatrixXd& priorC,
                                                        const MotionModelInterface* motionMdl, const unsigned int& N))
        {
            motionModel = motionMdl;
            priorMean = priorM;
            priorCovar = priorC;
            Num=N;
        }
    
    std::vector<Eigen::VectorXd> GenLinearStateSequences:: generateStateSequence()
        {
            auto n = x.rows();
            std::vector<Eigen::VectorXd> stateSequences;
            // initial sample
            Mvn mvn(priorMean, priorCovar,200);
            Eigen::VectorXd sampleVec= mvn.sample();
            stateSequences.push_back(sampleVec);

            // Zero mean random noise distribution
            Eigen::VectorXd mean(n);
            mean.setZero()
            Mvn mvnZeroMean(mean, priorCovar,motionModel.getProcessNoiseCovariance());

            for (unsigned i = 1; i < Num; i++)
            {
                stateSequences[i] = motionModel.getStateTransitionMatrix()*stateSequences[i-1] 
                                                                            + mvnZeroMean.sample();
            }

        }
}