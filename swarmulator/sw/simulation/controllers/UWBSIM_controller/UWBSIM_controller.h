#ifndef UWBSIM_CONTROLLER_H
#define UWBSIM_CONTROLLER_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "controller.h"
#include "UWBSIM_controller/ekf_state_estimator.h"
#include "randomgenerator.h"
#include <fstream>
#define COMMAND_LOCAL 1

class UWBSIM_controller: public Controller
{
	std::ifstream inFile1;
	//std::ifstream inFile2;
	//std::ifstream inFile3;
	//std::ifstream inFile4;
	//std::ifstream inFile5;
	
	
public:
    //ekf_state_estimator filter;
	ekf_state_estimator filter; 
	bool init; 
	float next_trajectory_time ;
	float next_EKF_measurement_time ;
	
	UWBSIM_controller();
	virtual void get_velocity_command(const uint16_t ID, float &v_x, float &v_y);
	virtual void animation(const uint16_t ID);
};


#endif /*UWBSIM_CONTROLLER_H*/
