#include<Eigen/Dense>
#include <random>
#include <vector>

#ifndef MVN
#define MVN

namespace dataSimulator
{
    class Mvn
    {
        public:
            Mvn(const Eigen::VectorXd& mu,const Eigen::MatrixXd& s,const unsigned int& size);
            ~Mvn(){};
            double pdf(const Eigen::VectorXd& x) const;
            Eigen::VectorXd sample() const;
        private:    
            Eigen::VectorXd mean;
            Eigen::MatrixXd sigma;
            unsigned int sampleSize;
    };
}
#endif