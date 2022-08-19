#include "quadrotor.h"
#include "trigonometry.h"
#include "draw.h"
#include "random"
#include "fmat.h"
#include "trigonometry.h"
#include "auxiliary.h"
#include "math.h"
#include <vector>
#include <random>

 
quadrotor::quadrotor(int i, std::vector<float> s, float tstep)
{
  state = s;
  ID = i;
  dt = tstep;

  orientation = 0.0;
  controller->set_saturation(1.0);
  state_estimate = s;
  imu_state_estimate = s;
  ekf_estimate = s;
  Cov = s; 
  state_desired_traj = s;
  UWBm = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
  controller_states = {0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0, 0.0, 0.0};
  noisetest = {0.0, 0.0, 0.0,0.0, 0.0, 0.0,0.0, 0.0, 0.0};
  next_IMU_measurement_time = simtime_seconds;
  DesiredX = state[0];
  DesiredY = state[1];
  x = state[0];
  y = state[1];
  //x = state[0];
  //y = state[1];
 
}
std::vector<float> quadrotor::state_update(std::vector<float> state)
{

  
  std::default_random_engine generator;
  std::normal_distribution<float> distribution(0,0.01);
  float noise_x = distribution(generator);
  float noise_y = distribution(generator);

  std::random_device rd;     // only used once to initialise (seed) engine
  std::mt19937 gen(rd());    // random-number engine used (Mersenne-Twister in this case)
  std::normal_distribution<float> dis(0, param->noise_motor_sigma());
  
if(simtime_seconds>=next_IMU_measurement_time){
next_IMU_measurement_time = next_IMU_measurement_time + param->IMU_timestep() ;
//update our state using IMU data (add noise on acceleration term)
  // Initialise our state without noise (1x)
  if(param->terminaloutput()==1.0){
  std::cout<<"IMU UPDATE"<<std::endl;
  }
  //std::random_device rd;     // only used once to initialise (seed) engine
  //std::mt19937 gen(rd());    // random-number engine used (Mersenne-Twister in this case)
  //std::normal_distribution<float> dist(0, param->acc_noise_sigma());
  
   
  float ax = state[4] + noise_x; 
  float ay = state[5] + noise_y;
  
  imu_state_estimate[4] = ax; // Acceleration x
  imu_state_estimate[5] = ay;// Acceleration y

  imu_state_estimate[2] += imu_state_estimate[4]*dt; // Acceleration x
  imu_state_estimate[3] += imu_state_estimate[5]*dt;// Acceleration y

  imu_state_estimate[0] += imu_state_estimate[2]*dt; // Acceleration x
  imu_state_estimate[1] += imu_state_estimate[3]*dt;// Acceleration 0


  //std::cout<<ID<<" "<<imu_state_estimate[4]<<" "<<state[4]<<" "<<imu_state_estimate[4]-state[4]<<" "<<noise<<std::endl;
  }
  //controller->saturate(v_x);
  //controller->saturate(v_y);
  //moving = controller->moving;
  float desx ; 
  float desy ; 
  
  controller->get_velocity_command(ID, desx, desy);
   // y+ towards East
  //beacon->dynamic_beacon_update(ID);
  beacon->measurement(ID);
  //float vxr, vyr;
  //rotate_xy(v_x, v_y, orientation, vxr, vyr);

  //generate desired trajectory to fly with our quadrotor based on inputs of velocity controller

  //input desired position to our quadrotor eom
  //DesiredX = state_desired_traj[0];
  //DesiredY = state_desired_traj[1];
 //if(init == false){
 //  s[ID]->state_estimate[0] = 0;
//   s[ID]->state_estimate[1] = 0;
 //  s[ID]->state_estimate[2] = 0;
 //  s[ID]->state_estimate[3] = 0;
 //  init = true; 
 //}
  DesiredX = desx; 
  DesiredY = desy;
  DesiredZ = 0;
  

  if(param->terminaloutput()==1.0){
  std::cout<<DesiredX <<" desired X "<<DesiredY<<"Desired Y "<<std::endl;
  }
  // torque input for motors (Ux) x direction
  float Kpx = param->Kpx(); 
  float Kdx = param->Kdx(); 
  float Kix = param->Kix(); 
 
  float k_px = DesiredX - state_estimate[0]; 
  float k_dx = dDesiredX - state_estimate[2];


  if(k_px > 1.0){
    k_px = 1.0; 
  }
   if(k_px < -1.0){
    k_px = -1.0; 
  }

 
  float Ux = Kpx*( k_px ) + Kdx*( k_dx ) + Kix*errorSumX ;

  errorSumX = errorSumX + ( k_px);

  dDesiredX = ( DesiredX - DesiredXpre ) / dt;
  DesiredXpre = DesiredX;
  ddDesiredX = ( dDesiredX - dDesiredXpre ) / dt;

  dDesiredXpre = dDesiredX;

  // torque input for motors (Uy) y direction
  float Kpy = param->Kpy(); 
  float Kdy = param->Kdy(); 
  float Kiy = param->Kiy();


  float k_py = DesiredY - state_estimate[1]; 
  float k_dy = dDesiredY - state_estimate[3];


  if(k_py > 1.0){
    k_py = 1.0; 
  }
   if(k_py < -1.0){
    k_py = -1.0; 
  }



  float Uy = Kpy*(k_py) + Kdy*( k_dy) + Kiy*errorSumY ;

  errorSumY = errorSumY + ( k_py);

  dDesiredY = ( DesiredY - DesiredYpre ) / dt;
  DesiredYpre = DesiredY;
  ddDesiredY = ( dDesiredY - dDesiredYpre ) / dt;
  dDesiredYpre = dDesiredY;


  // torque input for motors (U1) throttle
  float Kpz = param->Kpz(); 
  float Kdz = param->Kdz(); 
  float Kiz = param->Kiz(); 

  float U1 = Kpz*( DesiredZ - z ) + Kdz*( dDesiredZ - dz ) + Kiz*errorSumZ + dis(gen);
  errorSumZ = errorSumZ + ( DesiredZ - z );

  dDesiredZ = ( DesiredZ - DesiredZpre ) / dt;
  DesiredZpre = DesiredZ;

  if(isnan(wrapToPi_f(((m/U1)/((sin(Psi)*sin(Psi))+(cos(Psi)*cos(Psi))))* ((Ux*sin(Psi))-(Uy*cos(Psi))))/2)){
DesiredPhi = -Uy/g; 
  }else{
DesiredPhi = wrapToPi_f(((m/U1)/((sin(Psi)*sin(Psi))+(cos(Psi)*cos(Psi))))* ((Ux*sin(Psi))-(Uy*cos(Psi))))/2;
  }

  if(isnan(wrapToPi_f(((m/U1)/((sin(Psi)*sin(Psi))+(cos(Psi)*cos(Psi))))* ((Ux*cos(Psi))+(Uy*sin(Psi)))))){
DesiredTheta = Ux/g;
  }else{
    DesiredTheta = wrapToPi_f(((m/U1)/((sin(Psi)*sin(Psi))+(cos(Psi)*cos(Psi))))* ((Ux*cos(Psi))+(Uy*sin(Psi))));
  }



  // desired phi and theta
 //if(initialised == false){
  //  DesiredPhi = -Uy/g; 
  //  DesiredTheta = Ux/g;
  //  initialised = true; 
  //}else{
   // std::cout<<"YEEEEEYYYYYYYYY"<<std::endl;
    
  

  
  
 
 //DesiredTheta = wrapToPi_f(((m/U1)/((sin(Psi)*sin(Psi))+(cos(Psi)*cos(Psi))))* ((Ux*cos(Psi))+(Uy*sin(Psi))));

   if(DesiredPhi > 2*0.0872665){
     DesiredPhi = 2*0.0872665;
     
   }
    if(DesiredTheta > 2*0.0872665){
     DesiredTheta = 2*0.0872665;
   }

 if(DesiredPhi < -2*0.0872665){
     DesiredPhi = -2*0.0872665;
   }
    if(DesiredTheta < -2*0.0872665){
     DesiredTheta = -2*0.0872665;
   }


 
 
  // calculate U2 roll
  float KpP = param->Kpphi();
  float KiP = param->Kiphi();
  float KdP =param->Kdphi();
  
  float U2 = KpP*( DesiredPhi - Phi ) + KdP*( dDesiredPhi - dPhi )  + KiP*errorSumPhi + dis(gen);
  errorSumPhi = errorSumPhi + ( DesiredPhi - Phi );

  dDesiredPhi = ( DesiredPhi - DesiredPhipre ) / dt;
  DesiredPhipre = DesiredPhi;
  
  
  // calculate U3 pitch
  float KpT = param->Kptheta();
  float KiT = param->Kitheta();
  float KdT = param->Kdtheta();
  
  float U3 = KpT*( DesiredTheta - Theta ) + KdT*( dDesiredTheta - dTheta ) + KiT*errorSumTheta + dis(gen);
  errorSumTheta = errorSumTheta + ( DesiredTheta - Theta );

  dDesiredTheta = (DesiredTheta-DesiredThetapre)/dt;
  DesiredThetapre = DesiredTheta;

  
  // calculate U4 yaw
  float KpS = 5; 
  float KdS = 0.0001; 
 // float KiS = 0;

  float U4 = KpS*(0- Psi ) + KdS*( 0 - dPsi ) + dis(gen);
  // now lets solve the ODE's of the quadrotor

  // x direction
  ddx = (U1/m)*(cos(Phi)*sin(Theta)*cos(Psi) + sin(Phi)*sin(Psi));//(Theta*cos(Psi) + Phi*sin(Psi) );//

  

  // y direction 
  ddy = (U1/m)*(sin(Theta)*sin(Psi)*cos(Phi) - sin(Phi)*cos(Psi));//(Theta*sin(Psi) - Phi*cos(Psi));//

  float threshold = 10.0;
 float accx = ddx;
 float accy = ddy; 

  if(accx > threshold){
    accx = threshold; 
  }
   if(accx < -threshold){
    accx = -threshold; 
  }
  if(accy > threshold){
    accy = threshold; 
  }
   if(accy < -threshold){
    accy = -threshold; 
  }
dx = dx + accx*dt;
dy = dy + accy*dt;

float vthreshold = 3;
 float vvx = dx;
 float vvy = dy; 

  if(vvx > vthreshold){
    vvx = vthreshold; 
  }
   if(vvx < -vthreshold){
    vvx = -vthreshold; 
  }
  if(vvy > vthreshold){
    vvy = vthreshold; 
  }
   if(vvy < -vthreshold){
    vvy = -vthreshold; 
  }
 
  
  x = x + vvx*dt;
  y = y + vvy*dt;

  // z direction 
  ddz = (cos(Theta)*cos(Phi)*(U1/m))-g;

  dz = dz + ddz*dt;
  z = z + dz*dt;

  // phi direction 
  ddPhi =(L/Jx)*U2; 
  dPhi = dPhi + ddPhi*dt;
  Phi = Phi + dPhi*dt;

  // theta direction 
  ddTheta =(L/Jy)*U3; 
  dTheta =dTheta +ddTheta*dt;
  Theta = Theta +dTheta*dt;

  //psi direction 
  ddPsi = (1/Jz)*U4;
  dPsi = dPsi + ddPsi*dt;
  Psi = Psi + dPsi*dt;
  
  controller_states.at(0) = DesiredX; 
  controller_states.at(1) = DesiredY; 
  controller_states.at(2) = DesiredZ; 
  controller_states.at(3) = x; 
  controller_states.at(4) = y; 
  controller_states.at(5) = z; 
  controller_states.at(6) = Theta; 
  controller_states.at(7) = Phi; 
  controller_states.at(8) = Psi; 

  // desired trajectory acceleration
  state_desired_traj.at(4) =  ddDesiredX; // Acceleration x
  state_desired_traj.at(5) =  ddDesiredY; // Acceleration y
  // desired trajectory velocity
  state_desired_traj.at(2) = dDesiredX; // Velocity x
  state_desired_traj.at(3) = dDesiredY; // Velocity y
  // desired trajectory position

  state_desired_traj.at(0) = DesiredX; // Position x
  state_desired_traj.at(1) = DesiredY; // Position y

  //update our states 
  //actual attained state of the quadrotor according to linearised dynamics
 

  // Acceleration
  state.at(4) = accx; // Acceleration x
  state.at(5) = accy; // Acceleration y
 
  // Velocity
  state.at(2) = vvx; // Velocity x
  state.at(3) = vvy; // Velocity y

  // Position
  state.at(0) = x; // Position x
  state.at(1) = y; // Position y



  return state;
}

void quadrotor::animation()
{
 
}
