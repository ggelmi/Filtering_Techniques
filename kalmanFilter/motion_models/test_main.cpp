#include<Eigen/Dense>
#include<iostream>
#include"ConstantVelocityModel.h"

int main()
{

const double delta = 1;
const double sigma = 0.5;
const unsigned int dim = 4;

ConstantVelocityModel  cv_model(delta,sigma,dim);

std::cout << "The dimension is : " << cv_model.getDimension();

  



return 0;
}
