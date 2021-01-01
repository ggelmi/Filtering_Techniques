#include<iostream>
#include<iomanip>
#include<vector>
#include"Map.h"
#include<fstream>
#include"utility_functions.h"


using namespace utility;

int main()
{
    std::cout << "Hey" << std::endl;
    std::ifstream input;
    //input.open("/home/guuto/filtering/Filtering_Techniques/data/pfData/map_data.txt");
    Map* mp = new Map();
    std::string map_filename ("/home/guuto/filtering/Filtering_Techniques/data/pfData/map_data.txt");
    // Loading the landmark data to the map
    bool ld = load_map_data(map_filename,mp->landmark_list);
    std::cout << "The size of the landmarks in the map : " << mp->landmark_list.size()<< std::endl;
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
    
    std::ostringstream file;
    file << "/home/guuto/filtering/Filtering_Techniques/data/pfData/observation/observations_"<<std::setfill('0')<<std::setw(6) << 1 << ".txt";
    std::vector<utility::observation> obsVec;
    bool ld4 = load_obs_data(file.str(),obsVec);
    std::cout << "The size of the obs vector : " << obsVec.size()<< std::endl;
    

}