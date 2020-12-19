#include<Eigen/Dense>
#include"/home/guuto/octave/Filtering_Techniques/stateEstimation/motion_models/MotionModelInterface.h"
#include"/home/guuto/octave/Filtering_Techniques/stateEstimation/motion_models/ConstantVelocityModel.h"

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
                                    const MotionModelInterface* motionMdl,const unsigned int& N);

            Eigen::MatrixXd generateStateSequence();

            //void loadToFile();

        private:
            MotionModelInterface* motionModel;
            Eigen::VectorXd priorMean;
            Eigen::MatrixXd priorCovar;
            unsigned int Num;
    };

}
#endif