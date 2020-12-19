
#include<cmath>
#include"Mvn.h"

namespace dataSimulator
{
    
    Mvn::Mvn(const Eigen::VectorXd& mu,const Eigen::MatrixXd& s, const unsigned int& size)
        {
            mean = mu;
            sigma = s;
            sampleSize = size;
        }
    
    
    double Mvn::pdf(const Eigen::VectorXd& x) const
        {
            auto n = x.rows();
            double sqrt2pi = std::sqrt(2 * M_PI);
            double quadform  = (x - mean).transpose() * sigma.inverse() * (x - mean);
            double norm = std::pow(sqrt2pi, - n) *
                            std::pow(sigma.determinant(), - 0.5);
            return norm * exp(-0.5 * quadform);
        }


    Eigen::VectorXd Mvn::sample() const
        {
            auto n = mean.rows();

            // Generate x from the N(0, I) distribution
            Eigen::VectorXd x(n);
            Eigen::VectorXd sum(n);
            sum.setZero();
            for (unsigned int i = 0; i < sampleSize; i++)
                {
                    x.setRandom();
                    x = 0.5 * (x + Eigen::VectorXd::Ones(n));
                    sum = sum + x;
                }
            sum = sum - (static_cast<double>(sampleSize) / 2) * Eigen::VectorXd::Ones(n);
            x = sum / (std::sqrt(static_cast<double>(sampleSize) / 12));

           // Find the eigen vectors of the covariance matrix
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(sigma);
            Eigen::MatrixXd eigenvectors = eigen_solver.eigenvectors().real();

          // Find the eigenvalues of the covariance matrix
            Eigen::MatrixXd eigenvalues = eigen_solver.eigenvalues().real().asDiagonal();
  
          // Find the transformation matrix
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(eigenvalues);
            Eigen::MatrixXd sqrt_eigenvalues = es.operatorSqrt();
            Eigen::MatrixXd Q = eigenvectors * sqrt_eigenvalues;

            return Q * x + mean;
        }   

}