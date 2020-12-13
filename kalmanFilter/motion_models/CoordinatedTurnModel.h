#ifndef COORDINATED_TURN_MODEL_H
#define COORDINATED_TURN_MODEL_H

#include"MotionModelInterface.h"

class CoordinatedTurnModel: public  MotionModelInterface
{

  public:

   CoordinatedTurnModel(const double& dtime, const double& sigPhi, const double& sigOmega, unsigned int& dimension);

   ~CoordinatedTurnModel(){ };

   unsigned int getDimension() const ;

   Eigen::VectorXd predictState( const Eigen::VectorXd& currState);

   Eigen::MatrixXd getStateTransitionMatrix(const Eigen::VectorXd& currState) ;

  // Eigen::MatrixXd  getProcessNoiseMatrix() ;


   private:

    double dt;

    double sigmaPhi;

    double sigmaOmega;

    unsigned int DIM;





};

#endif 