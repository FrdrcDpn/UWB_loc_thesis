#ifndef BEACON_THREAD_H
#define BEACON_THREAD_H

#include <numeric>
#include <string>
#include <functional>
#include <cctype>
#include <algorithm>
#include <condition_variable>
#include <chrono>
#include "draw.h"
#include "settings.h"
#include "randomgenerator.h"
#include "environment.h"
#include "terminalinfo.h"
#include "auxiliary.h"
#include "beacon_gen.h"

/**
 * Update the beacon simulation
 * @param ID_b The ID of the beacon
 */
void run_beacon_simulation_step(const int &ID_b)
{
  while (program_running) {
    bool ready = (b.size() == 8+nagents && s.size() == nagents );
    if (ready) {
      mtx_bcn.lock_shared();
      // put here the beacon update functions
     std::vector<float> b_0 = b.at(ID_b)->state_b;
     std::vector<float> b_n = b.at(ID_b)->beacon_dynamic_state_update(b_0); // State update for dynamic beacons
     std::vector<float> b_p = b.at(ID_b)->beacon_status_update(b_n); // State update
     b.at(ID_b)->state_b = b_p; // Update
     mtx_bcn.unlock_shared();
     
        // Run the local clock
      // If param->simulation_realtimefactor()=0, which can be set in conf/parameters.xml, the thread will not sleep and just run at full speed.
      if (param->simulation_realtimefactor() > 0) {
        int t_wait = (int)1e6 / (param->simulation_updatefreq() * param->simulation_realtimefactor());
        std::this_thread::sleep_for(std::chrono::microseconds(t_wait));
      }}
}
}

/**
 * Generates new beacon + simulation thread at given position x0 y0
 *
 * @param ID_b The ID of the beacon
 * @param x Initial position of the beacon in x
 * @param y Initial position of the beacon in y
 */
void create_new_beacon(const int &ID_b, const std::vector<float> &states)
{
  // Initiate a new beacon. The define "BEACON" is defined at build time.
  // No need to change this part of the code if you want to use a different agent, just compile swarmulator with:
  // >> make BEACON=myawesomebeacon
  // By default, BEACON=beacon_tdoa
  
  mtx_bcn.lock();
  b.push_back(new Beacon_gen(ID_b, states));
  mtx_bcn.unlock();

  // Info message
  std::stringstream ss;
  ss << "BEACON " << ID_b << " initiated";
  terminalinfo::info_msg(ss.str());

  // Initiate the thread that controls the beacon
  std::thread Beacon_gen(run_beacon_simulation_step, ID_b);

  // Detach thread so that it runs independently
  Beacon_gen.detach();
}
#endif /*BEACON_THREAD_H*/
