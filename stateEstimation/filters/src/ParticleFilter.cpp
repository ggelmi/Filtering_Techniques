#include"ParticleFilter.h"
#include<random>

namespace stateEstimation_pf
{
    ParticleFilter::ParticleFilter(unsigned int numP, unsigned int numI)
        {
            numParticles = numP;
            numIterations = numI;
            isInitialized = false;
        }
        
    void ParticleFilter:: initialize(Eigen::VectorXd mean, Eigen::MatrixXd covariance)
        {
            weights.resize(numParticles);
            particles.resize(numIterations);

            std::normal_distribution<double> distrib_x(mean(1),covariance(1,1));
            std::normal_distribution<double> distrib_y(mean(2),covariance(2,2));
            std::normal_distribution<double> distrib_theta(mean(3),covariance(3,3));

            std::default_random_engine rnd_gen;

            for(int i=0; i<numParticles; i++)
            {
                Particle particle;
                particle.id = i;
                particle.x = distrib_x(rnd_gen);
                particle.y = distrib_y(rnd_gen);
                particle.theta = distrib_theta(rnd_gen);
                particle.weight = 1;

                particles[i] = particle;
                weights[i] = particle.weight;
            }

            isInitialized = true;
        }
        
}