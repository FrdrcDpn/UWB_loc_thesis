#ifndef EKF_STATE_ESTIMATOR_H
#define EKF_STATE_ESTIMATOR_H
#include "controller.h"
#include "ekf.h"


class ekf_state_estimator
{
  
public:
  struct speed speed; 
  struct pos pos;
  struct cov cov; 
  struct cov coolcov; 
  bool initialized;
  uint16_t ID;
  float simtime_seconds_store = 0;
  ekf filterekf;
  ekf_state_estimator();
  ~ekf_state_estimator() {};
  void init_ekf_filter();
  void run_ekf_filter();
  void run(uint16_t ID_in);
  float tdoa_t; 
  float tdoa_t_stored;
};

#endif /*EKF_STATE_ESTIMATOR_H*/
