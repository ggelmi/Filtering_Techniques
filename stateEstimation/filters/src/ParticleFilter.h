#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include"utility_functions.h"

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

        void initialize();
        
}
