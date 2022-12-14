#ifndef SIMULATION_THREAD_H
#define SIMULATION_THREAD_H

#include <numeric>
#include <cctype>
#include <algorithm>
#include <thread>
#include <cstdlib> // system, NULL, EXIT_FAILURE
#include <iostream>
#include <sstream> // std::stringstream, std::stringbuf
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "main.h"
#include "randomgenerator.h"
#include "terminalinfo.h"
#include "beacon_thread.h"
#include "agent_thread.h"
#include "drawingparams.h"
#include "settings.h"
#include "environment.h"
#include "fitness_functions.h"
#include "fifo.h"
#include <string>
/**
 * Extract the number of agents from the argument list.
 * Else, return an error.
 *
 * @param argc Number of arguments from terminal input when launching swarmulator
 * @param argv Content of arguments from terminal input when launching swarmulator
 */
void read_argv(int argc, char *argv[])
{
  if (argc <= 1) {
    terminalinfo::error_msg("Please specify the number of agents.");
  } else {
    nagents = std::stoi(argv[1]);
  }
}

/**
 * This function initiates the simulation.
 * All agents in the beginning initiate randomly with a mean position around the (0,0) point.
 * Once the vector of agents is created, each agent is launched in a separate thread.
 *
 * @param argc Number of arguments from terminal input when launching swarmulator
 * @param argv Content of arguments from terminal input when launching swarmulator
 */
void main_simulation_thread(int argc, char *argv[], std::string id)
{
  terminalinfo::info_msg("Simulation started.");
  read_argv(argc, argv); // Read the number of agents from the argument input
  random_generator rg;
  fifo f(id); // Open FIFO pipe

  for (uint16_t ID_b = 0; ID_b < 8; ID_b++) {
    if(ID_b == 0){
      // state= x location, y location, enabled, frequency, broadcasting, static/dynamic, ID, cov1, cov2, cov3, cov4
    std::vector<float> state_b = {param->x_beacon_1(), param->y_beacon_1(), param->beacon_1_en(), param->beacon_1_freq(), 0.0, 0.0, float(ID_b),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    create_new_beacon(ID_b, state_b); // Create a new beacon
    }
    if(ID_b == 1){
    std::vector<float> state_b = {param->x_beacon_2(), param->y_beacon_2(), param->beacon_2_en(), param->beacon_2_freq(), 0.0, 0.0, float(ID_b),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    create_new_beacon(ID_b, state_b); // Create a new beacon
    }
    if(ID_b == 2){
    std::vector<float> state_b = {param->x_beacon_3(), param->y_beacon_3(), param->beacon_3_en(), param->beacon_3_freq(), 0.0, 0.0, float(ID_b),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    create_new_beacon(ID_b, state_b); // Create a new beacon
    }
    if(ID_b == 3){
    std::vector<float> state_b = {param->x_beacon_4(), param->y_beacon_4(), param->beacon_4_en(), param->beacon_4_freq(), 0.0, 0.0, float(ID_b),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    create_new_beacon(ID_b, state_b); // Create a new beacon
    }
    if(ID_b == 4){
    std::vector<float> state_b = {param->x_beacon_5(), param->y_beacon_5(), param->beacon_5_en(), param->beacon_5_freq(), 0.0, 0.0, float(ID_b),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    create_new_beacon(ID_b, state_b); // Create a new beacon
    }
    if(ID_b == 5){
    std::vector<float> state_b = {param->x_beacon_6(), param->y_beacon_6(), param->beacon_6_en(), param->beacon_6_freq(), 0.0, 0.0, float(ID_b),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    create_new_beacon(ID_b, state_b); // Create a new beacon
    }
    if(ID_b == 6){
    std::vector<float> state_b = {param->x_beacon_7(), param->y_beacon_7(), param->beacon_7_en(), param->beacon_7_freq(), 0.0, 0.0, float(ID_b),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    create_new_beacon(ID_b, state_b); // Create a new beacon
    }
    if(ID_b == 7){
    std::vector<float> state_b = {param->x_beacon_8(), param->y_beacon_8(), param->beacon_8_en(), param->beacon_8_freq(), 0.0, 0.0, float(ID_b),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    create_new_beacon(ID_b, state_b); // Create a new beacon
    }
    
  }
  // Generate the random initial positions with (0,0) mean and 0.5 standard deviation
  if (nagents > 0) {
    float spread = environment.limits(); // Default
    std::vector<float> x0 = rg.uniform_float_vector(nagents, -spread, spread);
    std::vector<float> y0 = rg.uniform_float_vector(nagents, -spread, spread);
    std::vector<float> t0 = rg.uniform_float_vector(nagents, -M_PI, M_PI);

    // Check whether the robot is in the area, else fix.
    // Define rays extending beyond the maximum limits of the area (d) in all 4 directions, and use this to check whether the area is valid.
    std::vector<std::vector<float>> d(4);
    d[0] = {0., spread * 1.3f}; // 1.3 is arbitrary, just something > 1.0
    d[1] = {0., -spread * 1.3f};
    d[2] = { spread * 1.3f, 0.};
    d[3] = { -spread * 1.3f, 0.};

    uint16_t ID = 0;
    while (ID < nagents) {
      bool location_invalid = false;
      for (uint16_t dir = 0; dir < d.size(); dir++) {
        std::vector<float> s_n = {x0[ID], y0[ID]};
        if (environment.valid(ID, s_n, d[dir])) {
          location_invalid = true;
          break; // An agent initialized outside of valid area was found. Proceed to fix it.
        }
      }

      // Get a new location if invalid, else move on to test the next agent.
      if (location_invalid) {
        x0[ID] = rg.uniform_float(-spread, spread);
        y0[ID] = rg.uniform_float(-spread, spread);
      } else {
        ID++;
      }
    }
    
    
    // Generate the agents in the initial positions
    for (uint16_t ID = 0; ID < nagents; ID++) {
     
      t0[ID]= 0.0;
      

     
      std::ifstream inFile1;
      std::string trajectoryname = "sw/simulation/controllers/UWBSIM_controller/xyTrajectory" + std::to_string(ID+1) + ".txt";  
      inFile1.open(trajectoryname);
      if (!inFile1) {
        std::cout << "No trajectory file for agent " <<ID<<std::endl;
        exit(1); // terminate with error
      }else{
        inFile1 >> x0[ID] >> y0[ID]; 
        inFile1.close();
      }
      
      
      
      // Initial state vector
      // [position_x, position y, vel_x=0, vel_y=0, acc_x=0, acc_y=0, psi, psi_rate]
      //std::vector<float> state = {x0[ID], y0[ID], 0.0, 0.0, 0.0, 0.0, t0[ID], 0.0};
      
      std::vector<float> state = {x0[ID], y0[ID], 0.0, 0.0, 0.0, 0.0, t0[ID], 0.0};
      create_new_agent(ID, state); // Create a new agent

      // for every agent, another beacon thread is opened as well!! 
      // If dynamic beacons are enabled we will use the agent's estimate as beacon location!!

      // state= x location, y location, enabled, frequency, broadcasting, static/dynamic, ID, state estimate x, state estimate y , cov1, cov2, cov3, cov4, vx, vy
      std::vector<float> state_b = {x0[ID], y0[ID], param->dynamic_beacons(), param->beacon_dynamic_freq(), 0.0, 1.0, float(ID+8),0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0};
      create_new_beacon(ID+8, state_b); // Create a new beacon
    }
  }

  // Keep global clock running until swarmulator quits.
  // The clock is only used by the animation and the logger,
  // which use the clock of the first robot as a reference.
  // Robots to do not share this clock and operate in a detached manner.
  while (program_running) {
    if (!paused) {
      // Runtime finish evolution

      // If the parameter time_limit is set to 0 (in conf/parameters.xml) then the simulation runs indefinitely
      // Otherwise the simulation will quit after that time has passed, using the clock of the 0th robot as a reference.
      if (param->time_limit() > 0.0) {

        // Quit after a certain amount of time
        if (simtime_seconds > param->time_limit()) {

          mtx.lock(); // Lock main mutex
          mtx_bcn.lock();
          mtx_env.lock(); // Lock environment mutex
          terminalinfo::debug_msg("Sending message");
          f.send(evaluate_fitness()); // Send FIFO message
          mtx_env.unlock(); // Unlock environment mutex
          mtx_bcn.unlock();
          mtx.unlock(); // Unlock main mutex

          program_running = false; // Get ready to quit
        }
      }
    }
  };

}
#endif /*SIMULATION_THREAD_H*/
