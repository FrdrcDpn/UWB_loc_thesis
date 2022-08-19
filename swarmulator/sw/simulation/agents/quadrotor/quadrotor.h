#ifndef quadrotor_H
#define quadrotor_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "agent.h"

/**
 * This child class of agent implements the dynamics of simple accelerated quadrotors using a kinematic model
 */
class quadrotor: public Agent
{
public:
 
  float next_IMU_measurement_time;
  float maxacc = 0;
  float maxvel = 0;
  float nanfoundth = 0;
  float nanfoundph = 0;
  float x = 0;
  float dx = 0;
  float ddx = 0;
  float y = 0;
  float dy = 0;
  float ddy = 0;
  float z = 0;
  float dz = 0;
  float ddz = 0;
  float Phi = 0.0000001;
  float dPhi = 0;
  float ddPhi = 0;
  float Theta =0.0000001;
  float dTheta = 0;
  float ddTheta = 0;
  float Psi = 0;
  float dPsi = 0;
  float ddPsi = 0;
  float errorSumX = 0;
  float errorSumY = 0;
  float errorSumZ = 0;
  float errorSumPhi   = 0;
  float errorSumTheta = 0;
  float DesiredX = 0;
  float DesiredY = 0;
  float DesiredZ = 0;
  float dDesiredX = 0;
  float dDesiredY = 0;
  float ddDesiredX = 0;
  float ddDesiredY = 0;
  float dDesiredZ = 0;
  float DesiredXpre = 0;
  float DesiredYpre = 0;
  float dDesiredXpre = 0;
  float dDesiredYpre = 0;
  float DesiredZpre = 0;
  float DesiredPhi = 0;
  float dDesiredPhi = 0;
  float DesiredTheta = 0;
  float dDesiredTheta = 0;
  float DesiredPhipre = 0;
  float DesiredThetapre = 0;
  float maxTheta = 0; 
  float maxPhi = 0; 
  float maxPsi = 0; 
  //System Parameters:
  float m  = 0.022;     // mass (Kg)
  float L  = 0.042;//0.25;      // arm length (m)
  float Jx = 0.0000091914;//0.019688; // inertia seen at the rotation axis. (Kg.m^2)
  float Jy = 0.0000091914;//0.019688; // inertia seen at the rotation axis. (Kg.m^2)
  float Jz = 0.000022800;//0.039380; // inertia seen at the rotation axis. (Kg.m^2)
  float g  = 9.81;      // acceleration due to gravity m/s^2
 float prevx = 0;
 float prevy = 0; 
 float prevvx = 0;
 float prevvy = 0;
  float cprevx = 0;
 float cprevy = 0; 
 float cprevvx = 0;
 float cprevvy = 0;
   float maxacco = 0; 
  /**
   * Constructor
   */
  quadrotor(int i, std::vector<float> state, float tstep);

  // quadrotor eom value


  /**
   * State update implementation
   */
  std::vector<float> state_update(std::vector<float> state);

  /**
   * Animation openGL implementation for visualization
   */
  void animation();
};

#endif /*PARTICLE_H*/