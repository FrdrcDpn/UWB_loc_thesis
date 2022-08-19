#ifndef EKF_H
#define EKF_H
#include "controller.h"
#include "Eigen/Dense"

struct pos
{
   float x;
   float y;
   float z;
};

struct speed
{
   float vx;
   float vy;
   float vz;
};

struct cov
{
   float c1;
   float c2;
   float c3;
   float c4;
   float c5;
   float c6;
};

class ekf
{
  // let's first define our matrices
private:
 


public:
bool twr_performed = false; 
float kR = 1;
Eigen::MatrixXf NP{2,2};
 Eigen::MatrixXf U{6,1};
 Eigen::MatrixXf B{6,1};
 Eigen::MatrixXf X{6,1};
 Eigen::MatrixXf X_est{6,1};
  Eigen::MatrixXf X_cov{6,1};
  Eigen::MatrixXf X_stored{6,1};
  Eigen::MatrixXf I{6,6};
  Eigen::MatrixXf Z{4,1};
  Eigen::MatrixXf Zm{4,1};
  Eigen::MatrixXf dfdx{6,6};
  Eigen::MatrixXf dfdu{6,6};
  Eigen::MatrixXf P{6,6};
  Eigen::MatrixXf Pb{2,2};
  Eigen::MatrixXf Pa{2,2};
  Eigen::MatrixXf P1{2,2};
  Eigen::MatrixXf P1_est{2,2};
  Eigen::MatrixXf P1_CI{2,2};
  Eigen::MatrixXf X1{2,1};
  Eigen::MatrixXf X1_CI{2,1};
  Eigen::MatrixXf X1_est{2,1};
  Eigen::MatrixXf Xc{2,1};
  Eigen::MatrixXf P_opt{2,2};
   Eigen::MatrixXf P_est{6,6};
    Eigen::MatrixXf P_cov{6,6};
   Eigen::MatrixXf P_stored{6,6};
  Eigen::MatrixXf Q{6,6};
  Eigen::MatrixXf R{4,4};
   Eigen::MatrixXf R_cov{4,4};
  Eigen::MatrixXf dhdntwr{1,4};
  Eigen::MatrixXf dhdntdoa{1,4};
  Eigen::MatrixXf dhdn{4,4};
  Eigen::MatrixXf dhdx{4,6};
  Eigen::MatrixXf dhdxtdoa{1,6};
  Eigen::MatrixXf dhdxtwr{1,6};
  Eigen::MatrixXf K{6,4};

  ekf();
  
  ~ekf() {};
  void init_ekf(float dt, float posx, float posy, float velx, float vely, float accx, float accy);

  void ekf_set_state();

  void ekf_get_state(); 

  struct pos ekf_get_pos(); 

  struct speed ekf_get_speed();
  struct cov ekf_get_cov();

  void ekf_predict(uint16_t ID, float dt);
  

  void ekf_update_acc(float ax, float ay);

  void ekf_update_twr(float dist, float anchor_x, float anchor_y);
 void ekf_update_twr_CI(float dist, float anchor_x, float anchor_y, float Px, float Py,float Pxx, float Pyy);

  void ekf_update_tdoa(float dist, float anchor_0x, float anchor_0y, float anchor_1x, float anchor_1y); 
  void ekf_set_twr_noise(float s_twr);
   void ekf_set_tdoa_noise(float s_twr);
   void ekf_set_noise();
};

#endif /*EKF*/
