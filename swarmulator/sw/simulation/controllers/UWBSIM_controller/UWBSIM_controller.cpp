#include "UWBSIM_controller.h"
#include "main.h"
#include "draw.h"
#include "auxiliary.h"
#include <cmath>
#include <random>

#define SENSOR_MAX_RANGE 1.8
using namespace std;

UWBSIM_controller::UWBSIM_controller(): Controller()
{
   next_trajectory_time = 0;
	 next_EKF_measurement_time = simtime_seconds;
  set_max_sensor_range(SENSOR_MAX_RANGE);
  /*
    inFile1.open("sw/simulation/controllers/UWBSIM_controller/xyTrajectory1.txt");
    if (!inFile1) {
        cout << "Unable to open file";
        exit(1); // terminate with error
    }
  inFile2.open("sw/simulation/controllers/UWBSIM_controller/xyTrajectory2.txt");
    if (!inFile2) {
        cout << "Unable to open file";
        exit(1); // terminate with error
    }
     inFile3.open("sw/simulation/controllers/UWBSIM_controller/xyTrajectory3.txt");
    if (!inFile3) {
        cout << "Unable to open file";
        exit(1); // terminate with error
    }
    inFile4.open("sw/simulation/controllers/UWBSIM_controller/xyTrajectory4.txt");
    if (!inFile4) {
        cout << "Unable to open file";
        exit(1); // terminate with error
    }
    inFile5.open("sw/simulation/controllers/UWBSIM_controller/xyTrajectory5.txt");
    if (!inFile5) {
        cout << "Unable to open file";
        exit(1); // terminate with error
    }
*/
  //::ifstream inFile1;
  init = false; 
  
   
}

//actual controller codea
void UWBSIM_controller::get_velocity_command(const uint16_t ID, float &v_x, float &v_y)
{
  
if(init == false){
  std::string trajectoryname = "sw/simulation/controllers/UWBSIM_controller/xyTrajectory" + std::to_string(ID+1) + ".txt";  
  inFile1.open(trajectoryname);
  if (!inFile1) {
        std::cout << "No trajectory file for agent " <<ID<<std::endl;
        exit(1); // terminate with error
      }
  init = true; 
}

 
//if(simtime_seconds>=next_measurement_time){
  
  
 // s[ID]->imu_state_estimate = s[ID]->state_estimate;
  
  // Position
  //s[ID]->imu_state_estimate.at(0)+= s[ID]->imu_state_estimate[2]  * timestep + 0.5 * s[ID]->imu_state_estimate[4]  * pow(timestep, 2); // Position x
  //s[ID]->imu_state_estimate.at(1) += s[ID]->imu_state_estimate[3]  * timestep + 0.5 * s[ID]->imu_state_estimate[5]  * pow(timestep, 2); // Position y
 
   
//}//else{
  /* // Acceleration
  s[ID]->state_estimate.at(4) = acc_x; // Acceleration x
  s[ID]->state_estimate.at(5) = acc_y; // Acceleration y
  float timestep = 1.0/param->simulation_updatefreq();
  // Velocity
  s[ID]->state_estimate.at(2) += s[ID]->state_estimate[4] * timestep; // Velocity x
  s[ID]->state_estimate.at(3) += s[ID]->state_estimate[5] * timestep; // Velocity y
  // Position
  s[ID]->state_estimate.at(0)+= s[ID]->state_estimate[2]  * timestep + 0.5 * s[ID]->state_estimate[4]  * pow(timestep, 2); // Position x
  s[ID]->state_estimate.at(1) += s[ID]->state_estimate[3]  * timestep + 0.5 * s[ID]->state_estimate[5]  * pow(timestep, 2); // Position y
  */
//}
  
  
if(simtime_seconds>=next_EKF_measurement_time){
  next_EKF_measurement_time = next_EKF_measurement_time + param->EKF_timestep() ;
  filter.run(ID);

}

/*
if(s.size() == 4){
moving_1 = true;
moving_2 = true;
moving_3 = true;
moving_4 = true; 
}
if(s.size() == 3){
moving_1 = true;
moving_2 = true;
moving_3 = true;
moving_4 = true; 
}
if(s.size() == 2){
moving_1 = true;
moving_2 = true;
moving_3 = true;
moving_4 = true; 
}
if(s.size() == 1){
moving_1 = true;
moving_2 = true;
moving_3 = true;
moving_4 = true; 
}

if(ID == 4){
  if(simtime_seconds>=next_trajectory_time){
  next_trajectory_time = simtime_seconds + 1.0/param->trajectory_frequency() ;
  // while trajectory file is not end of file yet
  if(!inFile5.eof())  // EOF is false here
  {
      inFile5 >> v_x >> v_y; 
      
  }else{
    moving_1 = true;
    v_x = 0;
    v_y = -3;
    
  }
}}

if(ID == 3 && moving_1 == true){
  if(simtime_seconds>=next_trajectory_time){
  next_trajectory_time = simtime_seconds + 1.0/param->trajectory_frequency() ;
  // while trajectory file is not end of file yet
  if(!inFile4.eof())  // EOF is false here
  {
      inFile4 >> v_x >> v_y; 
      
  }else{
    moving_2 = true;
    v_x = 0;
    v_y = 3;
    
  }
}}

if(ID == 2 && moving_1 == true && moving_2 == true){
  if(simtime_seconds>=next_trajectory_time){
  next_trajectory_time = simtime_seconds + 1.0/param->trajectory_frequency() ;
  // while trajectory file is not end of file yet
  if(!inFile3.eof())  // EOF is false here
  {
      inFile3 >> v_x >> v_y; 
      
  }else{
    moving_3 = true;
    v_x = 3;
    v_y = 0;
    
  }
}}

if(ID == 1 && moving_1 == true && moving_2 == true && moving_3 == true ){
  if(simtime_seconds>=next_trajectory_time){
  next_trajectory_time = simtime_seconds + 1.0/param->trajectory_frequency() ;
  // while trajectory file is not end of file yet
  if(!inFile2.eof())  // EOF is false here
  {
      inFile2 >> v_x >> v_y;   
       
      
  }else{
    moving_4 = true;
    v_x = -3;
    v_y = 0;
    
  }

}}

if(ID == 0 && moving_1 == true && moving_2 == true && moving_3 == true && moving_4 == true){

if( simtime_seconds>=next_trajectory_time){
  next_trajectory_time = simtime_seconds + 1.0/param->trajectory_frequency() ; 
  // while trajectory file is not end of file yet
  if(!inFile1.eof())  // EOF is false here
  {
      inFile1 >> v_x >> v_y;    
      
  }else{
    v_x = -1.5;
    v_y = 1.5;
    
    program_running = false;
  }
}
}
*/
if( simtime_seconds>=next_trajectory_time){
  next_trajectory_time = simtime_seconds + 1.0/param->trajectory_frequency() ; 
  // while trajectory file is not end of file yet
if(!inFile1.eof())  // EOF is false here
  {
      inFile1 >> v_x >> v_y;    
      
  }else{  
    inFile1.close();
    program_running = false;

  }}
  if(param->terminaloutput()==1.0){
  std::cout<<v_x <<" desired X "<<v_y<<" Desired Y "<<std::endl;
  }
  
  if(param->terminaloutput()==1.0){
std::cout<<"VELOCITY CYCLE"<<std::endl;
  }
}

// animation of the controller and sensors
void UWBSIM_controller::animation(const uint16_t ID)
{

  draw d;
  d.circle_loop(SENSOR_MAX_RANGE);
 
}
