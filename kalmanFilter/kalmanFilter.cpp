#include<Eigen/Dense>
//#include<Eigen/Core>
#include<iostream>
#include<fstream>
#include<vector>

using namespace std ;
using namespace Eigen ;

void linearPrediction(Eigen::Vector2d& current_state, MatrixXd& current_cov, 
		      Eigen::Vector2d& predicted_state, MatrixXd& predicted_cov,
		      MatrixXd& A ,MatrixXd& Q)
	{
	
	// Computing the predicted state using the motion model
	predicted_state = A*current_state;
	// Computing the predicted covariance 
	predicted_cov = A*current_cov*A.transpose();

	}	      

void linearUpdate(Eigen::Vector2d& predicted_state, MatrixXd& predicted_cov,
		  Eigen::Vector2d& updated_state, MatrixXd& updated_cov,
		  Eigen::Vector2d& meas_vector, MatrixXd& H ,MatrixXd& R)
	{

	// Computing the innovation
	
        Eigen::Vector2d innovation = meas_vector - (H*predicted_state);
      
        std::cout << "innovation : \n" << innovation<< std::endl;

        //std::cout << "pred_cov Matrix : \n" << predicted_cov << endl;
	// Computing the predicted covariance of the measurement
	
	Eigen::MatrixXd S_K = (H*predicted_cov*H.transpose())+ R;
	
	std::cout << "pred_meas_cov Matrix : \n" << S_K << endl;
	
	// Computing the kalman gain
	
	Eigen::MatrixXd PH_transpose = predicted_cov*H.transpose();

	Eigen::MatrixXd K = PH_transpose * S_K.inverse();
	
	std::cout << "kalman gain Matrix : \n" << K << endl;
	// updating the state
	
	updated_state = predicted_state +  K*innovation;
        
	updated_cov = predicted_cov - (K*S_K*K.transpose());


	}
int main () {
        std::cout << " Eigen version : " << EIGEN_MAJOR_VERSION << " . "
                                    << EIGEN_MINOR_VERSION << endl ;

        std::ifstream input("/home/guuto/octave/Filtering_Techniques/data.mat");
	std::ofstream output("/home/guuto/octave/Filtering_Techniques/state_estimates.mat");

	int counter  = 0;
        double x,y;
        std::string line;
	int i = 0;
	std::vector<Eigen::Vector2d> measData;
	while (std::getline(input,line))
        {
	    if(counter > 4 && line != "")
            {
		std::istringstream iss(line);
		iss >> x >> y;
	    	//std::cout << line << std::endl;
	//	std::cout << "X = " << x << " "<< "Y= " << y << endl;
		Eigen::Vector2d meas_vector(x,y);
		measData.push_back(meas_vector);
		std:: cout << "measData : \n " << measData[i] << std::endl;
		i++;
	    }
	    counter++;
	}
	
	// Defining the prior state vector
	Eigen::Vector2d X_0(0,0);
	std::cout << "X_prior: \n"<< X_0<<std::endl;
	
	// Defining the Prior Covariance
	MatrixXd P_0(2,2);
        P_0(0,0) = 1;
        P_0(1,0) = 0;
        P_0(0,1) = 0;
        P_0(1,1) = 1;

	std::cout << "Prior_Covariance : \n" << P_0 << endl; 
        
	// Defining the process Model
	//
	MatrixXd A(2,2);
        A(0,0) = 1;
        A(1,0) = 0;
        A(0,1) = 1;
        A(1,1) = 1;
        
	std::cout << "Process Model Matrix : \n" << A << endl;
        
	// Defining the Process Noise
	//
	MatrixXd Q = P_0*20;

	std::cout << "Process Noise Matrix : \n" << Q << endl;
	
	// Defining the Measurement Model
        //
        MatrixXd H(2,2);
        H(0,0) = 1;
        H(1,0) = 0;
        H(0,1) = 0;
        H(1,1) = 1;

        std::cout << "Measurement Model Matrix : \n" << H << endl;

        // Defining the Process Noise
        //
        MatrixXd R = P_0*20;

        std::cout << "Measurement Noise Matrix : \n" << R << endl;

	int N = measData.size();
	std::cout << "The size of the Measurement data is " << N<< std::endl;
	
	// Defining a vector to hold the state data
	std::vector<Eigen::Vector2d> X;
        // Initializing the state vector with the prior
	X.push_back(X_0);

	// Defining a vector to hold the covariance data
	std::vector<Eigen::MatrixXd> P;
	// Initializing the cov vector with the prior covariance
	P.push_back(P_0);

	for (int i=0; i<N; i++)
	    {
		Eigen::Vector2d pred_state;
		MatrixXd pred_cov;

		Eigen::Vector2d curr_state = X[i];
                MatrixXd curr_cov = P[i];
                
		std::cout << "curr_state : \n" << curr_state << std::endl;

                std::cout << "curr_cov Matrix : \n" << curr_cov << endl;

		linearPrediction(curr_state, curr_cov, pred_state, pred_cov, A, Q);

		std::cout << "pred_state : \n" << pred_state << std::endl;

		std::cout << "pred_cov Matrix : \n" << pred_cov << endl;
		
		Eigen::Vector2d updated_state;
                MatrixXd updated_cov;

		// grab the current measurement
		Eigen::Vector2d curr_meas = measData[i];

		linearUpdate(pred_state, pred_cov, updated_state, updated_cov, curr_meas, H, R);

		std::cout << "updated_state : \n" << updated_state << std::endl;

                std::cout << "updated_cov Matrix : \n" << updated_cov << endl;

//		X.push_back(pred_state);
  //              P.push_back(pred_cov);

		X.push_back(updated_state);
		P.push_back(updated_cov);

		// Saving the state estimates into a file

		output << updated_state(0) << "      "<< updated_state(1) << std::endl;
	

	    }

return 0;

}

