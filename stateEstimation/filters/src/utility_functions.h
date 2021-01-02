#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include<iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <math.h>
#include<string>
#include "Map.h"

namespace utility
{

struct control_command
{
    double velocity;
    double yaw_rate;
};

struct observation
{
    double x;
    double y;
    int id;
};

struct ground_truth
{
    double x;
    double y;
    double theta;

};

inline double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline double * computeError(double gt_x, double gt_y, double gt_theta, double pf_x, double pf_y, double pf_theta) {
	static double error[3];
	error[0] = fabs(pf_x - gt_x);
	error[1] = fabs(pf_y - gt_y);
	error[2] = fabs(pf_theta - gt_theta);
	error[2] = fmod(error[2], 2.0 * M_PI);
	if (error[2] > M_PI) {
		error[2] = 2.0 * M_PI - error[2];
	}
	return error;
}

/* loads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */
bool load_map_data(const std::string& filename, std::vector<Map::landmark>& vec) 
    {

	    // Get file of map:
	    std::ifstream input_file;
		input_file.open(filename);
	    // Return if we can't open the file.
	    if(!input_file)
		{
			return false;
		}
	
	    // Declare single line of map file:
	    std::string line;

	    // Run over each single line:
	    while(getline(input_file, line))
            {

		        std::istringstream iss_object(line);

            
                // Declare landmark values and ID:
		        double landmark_x, landmark_y;
		        int id; 
                // Read data from current line to values::
		        iss_object >> landmark_x;
		        iss_object >> landmark_y;
		        iss_object >> id;
                // Declare single_landmark:
		        Map::landmark landmark_object;

		        // Set values
		        landmark_object.id = id;
		        landmark_object.x  = landmark_x;
		        landmark_object.y  = landmark_y;

		    	// Add to landmark list of map:      
		        vec.push_back(landmark_object);
            }
			
	    return true;
    }
bool load_control_data(const std::string& filename, std::vector<utility::control_command>& vec) 
    {

	    // Get file of map:
	    std::ifstream input_file;
		input_file.open(filename);
	    // Return if we can't open the file.
	    if(!input_file)
		{
			return false;
		}
	
	    // Declare single line of map file:
	    std::string line;

	    // Run over each single line:
	    while(getline(input_file, line))
            {

		        std::istringstream iss_object(line);

                
				// declare control values
				double velocity, yaw_rate;
				iss_object >> velocity;
		        iss_object >> yaw_rate;
				// declare control object
				utility::control_command contr_com;

				// set values of the control command
				contr_com.velocity = velocity;
				contr_com.yaw_rate = yaw_rate;

				// add it to the control vector
				vec.push_back(contr_com);

            }
			
	    return true;
    }
bool load_gt_data(const std::string& filename, std::vector<utility::ground_truth>& vec) 
    {

	    // Get file of map:
	    std::ifstream input_file;
		input_file.open(filename);
	    // Return if we can't open the file.
	    if(!input_file)
		{
			return false;
		}
	
	    // Declare single line of map file:
	    std::string line;

	    // Run over each single line:
	    while(getline(input_file, line))
            {

		        std::istringstream iss_object(line);

                
				// declare gt values
				double x,y,theta;
				iss_object >> x;
		        iss_object >> y;
				iss_object >> theta;
				// declare gt object
				utility::ground_truth gt;

				// set values of the control command
				gt.x = x;
				gt.y= y;
				gt.theta = theta;

				// add it to the control vector
				vec.push_back(gt);

            }
			
	    return true;
    }

bool load_obs_data(const std::string& filename, std::vector<utility::observation>& vec) 
    {

	    // Get file of map:
	    std::ifstream input_file;
		input_file.open(filename);
	    // Return if we can't open the file.
	    if(!input_file)
		{
			return false;
		}
	
	    // Declare single line of map file:
	    std::string line;

	    // Run over each single line:
	    while(getline(input_file, line))
            {

		        std::istringstream iss_object(line);

                
				// declare observation values
				double x,y;
				iss_object >> x;
		        iss_object >> y;
				// declare gt object
				utility::observation obs;

				// set values of the control command
				obs.x = x;
				obs.y= y;

				// add it to the control vector
				vec.push_back(obs);

            }
			
	    return true;
    }
}
#endif
