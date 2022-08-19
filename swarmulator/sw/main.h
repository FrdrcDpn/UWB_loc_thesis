#ifndef MAIN_H
#define MAIN_H

#include <mutex>
#include <shared_mutex>
#include <vector>
#include "beacon_gen.h"
#include "agent.h"
#include "parameters.hxx" // Auto generated file at compile time
#include "environment.h"
#include "settings.h"
extern bool moving_1;
extern bool moving_2;
extern bool moving_3;
extern bool moving_4;

extern std::vector<float> xpositions;
extern std::vector<float> ypositions;
extern std::vector<int> startposindex;
extern float tdoa_noise; 
extern float twr_noise; 
extern std::vector<std::vector<std::vector<float>>> beacon_measurement;
extern uint nagents; // Number of agents in the swarm
extern std::vector<Beacon_gen *> b; // Set up a vector of beacons
extern std::vector<Agent *> s; // Set up a vector of agents
extern float simtime_seconds; // Adjusted simulation time (time according to simulation)
extern std::shared_mutex mtx; // Mutex object
extern std::shared_mutex mtx_env; // Mutex object
extern std::shared_mutex mtx_bcn; // Mutex object
extern std::shared_mutex mtx_e; // Mutex object
extern bool program_running; // True if the program is (or should be) running. If false the program shuts down.
extern std::unique_ptr<parameters_t> param; // XML parameters from conf file
extern float realtimefactor; // Real time factor of simulation
extern Environment environment; // Vector holding the environment
extern std::string identifier; // String identifier for FIFO pipe
extern std::vector<std::vector<std::vector<float>>> UWB;
extern std::string beacon_alg; //what kind of beacon localisation algorithm is used
#endif /*MAIN_H*/
