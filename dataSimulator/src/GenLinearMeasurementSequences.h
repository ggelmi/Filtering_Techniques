#include<Eigen/Dense>
#include"MeasurementModelInterface.h"
#include"CvMeasurementModel.h"

#include"Mvn.h"

#ifndef GEN_LINEAR_MEASUREMENT_SEQUENCES
#define GEN_LINEAR_MEASUREMENT_SEQUENCES

using namespace stateEstimation;

namespace dataSimulator
{

    class GenLinearMeasurementSequences
    {
        public:
            GenLinearMeasurementSequences(MeasurementModelInterface*& measMdl);

            std::vector<Eigen::VectorXd> generateMeasurementSequence(const std::vector<Eigen::VectorXd>& stateSequences);

        private:
            MeasurementModelInterface* measModel;
    };

}
#endif