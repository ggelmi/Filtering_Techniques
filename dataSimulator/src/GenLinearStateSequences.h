#include<Eigen/Dense>
#include"MotionModelInterface.h"
#include"ConstantVelocityModel.h"

#include"Mvn.h"

#ifndef GEN_LINEAR_STATE_SEQUENCES
#define GEN_LINEAR_STATE_SEQUENCES

using namespace stateEstimation;

namespace dataSimulator
{

    class GenLinearStateSequences
    {
        public:
            GenLinearStateSequences(const Eigen::VectorXd& priorM,const Eigen::MatrixXd& priorC,
                                    MotionModelInterface*& motionMdl,const unsigned int& N);

            std::vector<Eigen::VectorXd> generateStateSequence();

        private:
            MotionModelInterface* motionModel;
            Eigen::VectorXd priorMean;
            Eigen::MatrixXd priorCovar;
            unsigned int Num;
    };

}
#endif