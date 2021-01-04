#include<iostream>
#include<iomanip>
#include<vector>
//#include"Map.h"
#include<fstream>
#include<random>
//#include"utility_functions.h"
#include"ParticleFilter.h"


using namespace utility;
using namespace stateEstimation_pf;


int main()
{
    std::cout << "Hey" << std::endl;
    std::ifstream input;
    //input.open("/home/guuto/filtering/Filtering_Techniques/data/pfData/map_data.txt");
    Map* map = new Map();
    std::string map_filename ("/home/guuto/filtering/Filtering_Techniques/data/pfData/map_data.txt");
    // Loading the landmark data to the map
    bool ld = load_map_data(map_filename,map->landmark_list);
    std::cout << "The size of the landmarks in the map : " << map->landmark_list.size()<< std::endl;
    // loading the control data
    std::vector<utility::control_command> controlVec;
    std::string cc_filename ("/home/guuto/filtering/Filtering_Techniques/data/pfData/control_data.txt");
    bool ld2 = load_control_data(cc_filename,controlVec);
    std::cout << "The size of the control : " << controlVec.size()<< std::endl;
    // loading the groundtruth data
    std::vector<utility::ground_truth> gtVec;
    std::string gt_filename ("/home/guuto/filtering/Filtering_Techniques/data/pfData/gt_data.txt");
    bool ld3 = load_gt_data(gt_filename,gtVec);
    std::cout << "The size of the ground_truth : " << gtVec.size()<< std::endl;
   
    double delta_t = 0.1;
    double sensor_range = 50;
    // GPS uncertainty x in meters, y in meters and theta in radians
    Eigen::MatrixXd sigma_gps(3, 3);
    sigma_gps <<   0.3, 0,0,
                    0, 0.3,0,
                    0,0, 0.01;
                
    Eigen::MatrixXd sigma_meas(2, 2);
    sigma_meas <<  0.3,0,
                    0, 0.3;
        
    
    std::default_random_engine rnd_gen;
    std::normal_distribution<double> rnd_x(0,sigma_gps(0,0));
    std::normal_distribution<double> rnd_y(0,sigma_gps(1,1));
    std::normal_distribution<double> rnd_theta(0,sigma_gps(2,2));

    std::normal_distribution<double> rnd_obs_x(0,sigma_meas(0,0));
    std::normal_distribution<double> rnd_obs_y(0,sigma_meas(0,0));
    
    unsigned int num_particles = 1000;
    unsigned int num_iterations = 10;
    
    ParticleFilter* pf = new ParticleFilter(num_particles,num_iterations);

    double rn_x, rn_y, rn_theta, rn_range, rn_heading;

    for(int i=1; i<num_iterations; i++)
    {
        std::ostringstream file;
        file << "/home/guuto/filtering/Filtering_Techniques/data/pfData/observation/observations_"<<std::setfill('0')<<std::setw(6) << i << ".txt";
        std::vector<utility::observation> obsVec;
        bool ld4 = load_obs_data(file.str(),obsVec);
        if(!ld4){std::cout << "Could not open the observation file" << std::endl;}
        std::cout << "The size of the obs vector : " << obsVec.size()<< std::endl;
        
        if(!pf->isInitialized)
        {
            int j = 0;
            rn_x = rnd_x(rnd_gen);
            rn_y = rnd_y(rnd_gen);
            rn_theta = rnd_theta(rnd_gen);

            std::cout << "rn_x: " << rn_x << std::endl;
            Eigen::VectorXd rnd_vector(3);
            rnd_vector << rn_x, rn_y, rn_theta;
            Eigen::VectorXd gt_vector(3);
            gt_vector << gtVec[j].x, gtVec[j].y,gtVec[j].theta;

            Eigen::VectorXd g = gt_vector + rnd_vector;

            pf->initialize(g,sigma_gps);
        }
        else
        {
            Eigen::VectorXd control_vector(2);
            control_vector << controlVec[i-1].velocity, controlVec[i-1].yaw_rate;
            pf->predict(control_vector,sigma_gps,delta_t);
        }

        // Given the noise measurement, we assume it was noiseless and we add noise to it
        std::vector<utility::observation> noisy_measurements;
        utility::observation obs;
        for(int j=0; j<obsVec.size(); i++)
        {
            rn_x = rnd_obs_x(rnd_gen);
            rn_y = rnd_obs_y(rnd_gen);
            obs = obsVec[j];
            obs.x = obs.x + rn_x;
            obs.y = obs.y + rn_y;
            noisy_measurements.push_back(obs);
        }

        // updating the weights of the samples
        pf->updateWeights(noisy_measurements, sigma_meas,map,sensor_range);
        
    }



}