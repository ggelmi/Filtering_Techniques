#include"ParticleFilter.h"
#include<random>
#include<iostream>

namespace stateEstimation_pf
{
    ParticleFilter::ParticleFilter(unsigned int numP, unsigned int numI)
        {
            numParticles = numP;
            numIterations = numI;
            isInitialized = false;
        }
        
    void ParticleFilter:: initialize(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance)
        {
            
            weights.resize(numParticles);
            particles.resize(numParticles);

            std::normal_distribution<double> distrib_x(mean(0),covariance(0,0));
            std::normal_distribution<double> distrib_y(mean(1),covariance(1,1));
            std::normal_distribution<double> distrib_theta(mean(2),covariance(2,2));

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
            std::cout << "The size of initial particles: " << particles.size() << std::endl;
        }
    void ParticleFilter::predict(const Eigen::VectorXd& controlVec,const Eigen::MatrixXd& covariance,const double& dt)
        {

            std::default_random_engine rnd_gen;
            double velocity = controlVec(0);
            double yaw_rate = controlVec(1);
            for(int i=0; i<numParticles; i++)
            {
                Particle* par = &particles[i];
                
                // Computing the new robot location based on kinematics and control commands
                double x_new = par->x + (velocity/yaw_rate) * (sin(par->theta + yaw_rate*dt) - sin(par->theta)) ;
                double y_new = par->y + (velocity/yaw_rate) * (cos(par->theta) - cos(par->theta + yaw_rate*dt )) ;
                double theta_new = par->theta + (yaw_rate*dt);

                // Adding gaussian noise to the predicted position
                std::normal_distribution<double> distrib_x(x_new,covariance(0,0));
                std::normal_distribution<double> distrib_y(y_new,covariance(1,1));
                std::normal_distribution<double> distrib_theta(theta_new,covariance(2,2));   

                // Updating the particle fields
                par->x = distrib_x(rnd_gen);
                par->y = distrib_y(rnd_gen);
                par->theta = distrib_theta(rnd_gen);
            }

        }
    
    void ParticleFilter:: updateWeights(const std::vector<utility::observation>& noisy_meas, const Eigen::MatrixXd& covariance, 
                        const Map* map, const double& sensor_range)
        {

            for(int j=0; j<noisy_meas.size(); j++)
            {
                std::cout << "Noisy meas : " << noisy_meas[j].x << std::endl;
            }
        }
    
        
}