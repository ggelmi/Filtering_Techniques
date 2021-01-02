#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include<Eigen/Dense>
#include<vector>

namespace stateEstimation_pf
{
    struct Particle
    {
        int id;
        double x;
        double y;
        double theta;
        double weight;
    };

    class ParticleFilter
    {
        public:
            
            ParticleFilter(unsigned int numP, unsigned int numI);

            ~ParticleFilter(){};
            // initialize the particle filter given the mean
            void initialize(Eigen::VectorXd mean, Eigen::MatrixXd covariance);

            bool isInitialized;

            //void runOneStep();

            
        private:
            /**
            void predict();

            void updateWeights();

            void resampleParticles();

            void dataAssociation();
            **/
            std::vector<Particle> particles;

            std::vector<double> weights;
            
            unsigned int numParticles;

            unsigned int numIterations;

        
    };


}
#endif