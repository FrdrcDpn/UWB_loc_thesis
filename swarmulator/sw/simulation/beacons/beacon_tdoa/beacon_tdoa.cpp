#include "beacon_tdoa.h"
#include "agent.h"
#include "main.h"
#include <stdio.h>
#include <boost/math/special_functions/gamma.hpp>
using namespace std;
#define EPSILON 0.001

beacon_tdoa::beacon_tdoa() {
beacon_alg = "beacon_tdoa";

}

/*
*in parameters file, change params file to define the noise type
*noise_type 0 = no noise
*noise_type 1 = gaussian noise
*noise_type 2 = heavy tailed cauchy noise
*noise_type 3 = heavy tailed gamma noise
*/
/*
// function to output some range info to terminal (not useful to be removed)
void beacon_tdoa::ranges_terminal(const uint16_t ID){
    //output ranges to all beacons in terminal
    float r1;
    //draw the range from the beacons
    cout << "agent id:"<< ID;
    for (uint16_t i = 0; i < 8; i++) {
        r1 = range_beacon(ID)[i];
        cout << "   range b"<< i+1 <<" "<< r1;
    }
    cout << endl;
}

// function to return some UWB data (not useful to be removed)
float beacon_tdoa::returnUWBdata(const uint16_t ID, float beacon){
   // function that returns the latest distance entry of uwb data
   mtx_bcn.lock();
   float dist = UWB[ID].back()[0];
   mtx_bcn.unlock();
   return dist;
}
*/
// measurement function, called by controller at simulation frequency
// constructs UWB measurement from available UWB beacons
void beacon_tdoa::measurement(const uint16_t ID){
    
    float x_0,y_0,x_1,y_1,dx0,dy0,dx1,dy1,d0,d1,dd,x_a_0,y_a_0,x_a_1,y_a_1;

    // initial values
    bool static_beacon_1 = false;
    bool dynamic_beacon_1 = false;
    bool static_beacon_2 = false;
    bool dynamic_beacon_2 = false;
    bool beacon_1_selected = false; 
    bool beacon_2_selected = false; 
  
    mtx_e.lock();
    // we make a copy of beacon state vector at moment of measurement (freeze in time)
    std::vector<Beacon_gen *> b_0 = b; 
    mtx_e.unlock();
    // Now we random shuffle the vector to make sure a random broadcasting beacon is selected
    std::random_shuffle (b_0.begin(), b_0.end() ); 

    // Now we loop over the shuffeled beacon state vector and we select 2 enabled and broadcasting beacons
    for (uint16_t ID_b = 0; ID_b < b_0.size(); ID_b++) {
        // first select beacon 1 
        
        if(b_0[ID_b]->state_b[4] == 1.0 && beacon_1_selected == false && ID+8!=b_0[ID_b]->state_b[6]){
           //if we have a static beacon
           if(b_0[ID_b]->state_b[5]==0.0){
            x_0 = b_0[ID_b]->state_b[0]; // x-location of static beacon from its state vector
            y_0 = b_0[ID_b]->state_b[1]; // y-location of static beacon from its state vector
            x_a_0 = x_0; // for the static beacons we use the static x location as anchor x coordinate
            y_a_0 = y_0; // for the static beacons we use the static y location as anchor y coordinate
           }else{
            x_0 = b_0[ID_b]->state_b[7]; // x-location of dynamic beacon from desired trajectory
            y_0 = b_0[ID_b]->state_b[8]; // y-location of dynamic beacon from desired trajectory
            x_a_0 = b_0[ID_b]->state_b[0]; // for the dynamic beacons we use the dymamic x state estimate as anchor x coordinate
            y_a_0 = b_0[ID_b]->state_b[1]; // for the dynamic beacons we use the dymamic y state estimate as anchor y coordinate
           }
           beacon_1_selected = true; // we have selected the beacon
           static_beacon_1 = !bool(b_0[ID_b]->state_b[5]); // 5th entry of state vector is 0.0 if it is a static beacon, 1.0 if it is dynamic
           dynamic_beacon_1 = bool(b_0[ID_b]->state_b[5]);
           sel_beacon_1 = int(b_0[ID_b]->state_b[6]); //6h entry of state vector is the beacon ID 
        }
        // now select beacon 2
        if(b_0[ID_b]->state_b[4] == 1.0 && beacon_1_selected == true && beacon_2_selected == false && sel_beacon_1 != int(b_0[ID_b]->state_b[6])&& ID+8!=b_0[ID_b]->state_b[6]){
           //if we have a static beacon
           if(b_0[ID_b]->state_b[5]==0.0){
            x_1 = b_0[ID_b]->state_b[0]; // x-location of static beacon from its state vector
            y_1 = b_0[ID_b]->state_b[1]; // y-location of static beacon from its state vector
            x_a_1 = x_1; // for the static beacons we use the static x location as anchor x coordinate
            y_a_1 = y_1; // for the static beacons we use the static y location as anchor y coordinate
           }else{
            x_1 = b_0[ID_b]->state_b[7]; // x-location of dynamic beacon from desired trajectory
            y_1 = b_0[ID_b]->state_b[8]; // y-location of dynamic beacon from desired trajectory
            x_a_1 = b_0[ID_b]->state_b[0]; // for the dynamic beacons we use the dymamic x state estimate as anchor x coordinate
            y_a_1 = b_0[ID_b]->state_b[1]; // for the dynamic beacons we use the dymamic y state estimate as anchor y coordinate
           }
           beacon_2_selected = true; // we have selected the beacon
           static_beacon_2 = !bool(b_0[ID_b]->state_b[5]); // 5th entry of state vector is 0.0 if it is a static beacon, 1.0 if it is dynamic
           dynamic_beacon_2 = bool(b_0[ID_b]->state_b[5]); 
           float sel_beacon_2 = int(b_0[ID_b]->state_b[6]); //6h entry of state vector is the beacon ID 
           if(param->terminaloutput()==1.0){
           std::cout<<"agent "<<ID<<" ranges with beacon "<<sel_beacon_1<<std::endl; // output the selected beacons to terminal (can be commented)
           std::cout<<"agent "<<ID<<" ranges with beacon "<<sel_beacon_2<<std::endl; // output the selected beacons to terminal (can be commented)
           }
        }}
   
   
    
    // if we have selected 2 available beacons during this measurement cycle, continue
    if(beacon_1_selected == true && beacon_2_selected == true && simtime_seconds>=next_UWB_measurement_time){
        next_UWB_measurement_time = next_UWB_measurement_time + 1.0/param->UWB_frequency();
    // generate tdoa measurements
    dx0 = s[ID]->state[0] - x_0;
    dy0 = s[ID]->state[1] - y_0;

    dx1 = s[ID]->state[0] - x_1;
    dy1 = s[ID]->state[1] - y_1;

    d0 = sqrt(dx0*dx0 + dy0*dy0);
    d1 = sqrt(dx1*dx1 + dy1*dy1);
    dd = d1-d0;

    //initialise uwb dataset to be used by EKF
   // mtx_bcn.lock();
   
    //UWB.push_back(std::vector<std::vector<float>>());
   // UWB[ID].push_back(std::vector<float>());
    
  //  mtx_bcn.unlock();
    //add preferred noise on top of tdoa measurements, defined in parameters file
    if (param->noise_type() == 0){
        // tdoa measurement, x beacon1, y beacon1, x beacon2, y beacon2, simulation time
       // mtx_e.lock();
        s[ID]->UWBm.at(0) = dd;
        s[ID]->UWBm.at(1) = x_a_0;
        s[ID]->UWBm.at(2) = y_a_0;
        s[ID]->UWBm.at(3) = x_a_1;
        s[ID]->UWBm.at(4) = y_a_1;
       // s[ID]->UWBm.at(5) = simtime_seconds;
       // UWB[ID].push_back({dd,, ,,, });
      //  mtx_e.unlock();
    }
    else if (param->noise_type() == 1){
        // tdoa measurement, x beacon1, y beacon1, x beacon2, y beacon2, simulation time
       // mtx_e.lock();
        s[ID]->UWBm.at(0) = add_gaussian_noise(dd, param->gauss_sigma_tdoa());
        s[ID]->UWBm.at(1) = x_a_0;
        s[ID]->UWBm.at(2) = y_a_0;
        s[ID]->UWBm.at(3) = x_a_1;
        s[ID]->UWBm.at(4) = y_a_1;
        //s[ID]->UWBm.at(5) = simtime_seconds;
       // UWB[ID].push_back({add_gaussian_noise(dd),x_a_0, y_a_0,x_a_1,y_a_1, simtime_seconds});
      //  mtx_e.unlock();
    }
    else if (param->noise_type() == 2){
        // tdoa measurement, x beacon1, y beacon1, x beacon2, y beacon2, simulation time
        //mtx_e.lock();
        s[ID]->UWBm.at(0) = add_ht_cauchy_noise(dd);
        s[ID]->UWBm.at(1) = x_a_0;
        s[ID]->UWBm.at(2) = y_a_0;
        s[ID]->UWBm.at(3) = x_a_1;
        s[ID]->UWBm.at(4) = y_a_1;
       // s[ID]->UWBm.at(5) = simtime_seconds;

        //UWB[ID].push_back({add_ht_cauchy_noise(dd),x_a_0, y_a_0,x_a_1,y_a_1,simtime_seconds});
        //mtx_e.unlock();
    }
    else if (param->noise_type() == 3){
        // tdoa measurement, x beacon1, y beacon1, x beacon2, y beacon2, simulation time
        s[ID]->UWBm.at(0) = add_ht_gamma_noise(dd);
        s[ID]->UWBm.at(1) = x_a_0;
        s[ID]->UWBm.at(2) = y_a_0;
        s[ID]->UWBm.at(3) = x_a_1;
        s[ID]->UWBm.at(4) = y_a_1;
       // s[ID]->UWBm.at(5) = simtime_seconds;

       // mtx_e.lock();
        //UWB[ID].push_back({add_ht_gamma_noise(dd),x_a_0, y_a_0,x_a_1,y_a_1, simtime_seconds});
       //mtx_e.unlock();
    }
    if(param->terminaloutput()==1.0){
    // output to terminal whether the selected beacons are dynamic or static (can be commented)
    std::cout<<"dynamic beacon_1 "<<dynamic_beacon_1<<" static beacon_1 "<<static_beacon_1<<std::endl;
    std::cout<<"dynamic beacon_2 "<<dynamic_beacon_2<<" static beacon_2 "<<static_beacon_2<<std::endl;
    }
    // if during this cycle a measurement is available, let the EKF know by pusing a 1 
   // mtx_e.lock();
    //beacon_measurement.push_back(std::vector<std::vector<float>>());
    //beacon_measurement[ID].push_back(std::vector<float>());
    s[ID]->UWBm.at(5) = 1 ;
   // beacon_measurement[ID].push_back({1});
   // mtx_e.unlock();
    
   }
}
   
// function to add gaussian noise to UWB measurements
float beacon_tdoa::add_gaussian_noise(float value, float sigma) {
    float noisy_value;
    float mean = 0;
  
    // Random seed
    random_device rd;

    // Initialize Mersenne Twister pseudo-random number generator
    mt19937 gen(rd());
    std::normal_distribution<double> dis(mean, sigma);
    // Add Gaussian noise
    noisy_value = value + dis(gen);

    return noisy_value;
}

//our cauchy cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_tdoa::cdf_ht_cauchy(double x)
{
   // generate our random variable
    float tmp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    //gaussian cdf
    float gauss = 0.5*(1+erf( x/(sqrt(2)*param->gauss_sigma()))) * (1-param->htc_ratio());
    //cauchy cdf
    float cauchy = (((1/M_PI) * atan(x/param->htc_gamma())) + 0.5) * param->htc_ratio();
    //return ht_cauchy cdf
    return gauss + cauchy - tmp;
}

// the derivative of our cauchy cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_tdoa::deriv_cdf_ht_cauchy(double x)
{
    float gauss = (1-param->htc_ratio())*(1/((sqrt(2*M_PI))*param->gauss_sigma()))*exp((-(x*x)/(2*param->gauss_sigma()*param->gauss_sigma())));
    //cauchy cdf
    float cauchy = (param->htc_gamma()*param->htc_ratio())/((M_PI*x*x)+(param->htc_gamma()*param->htc_gamma()*M_PI));
    //return ht_cauchy cdf
    return gauss + cauchy;
}

//our ht_cauchy noise function, that solves for heavy tailed noise using Newton Rapson's method
float beacon_tdoa::add_ht_cauchy_noise(float value){

    //our starting estimate to solve the cdf using Newton Raphson's method
    double x = 0; 
    double h = cdf_ht_cauchy(x) / deriv_cdf_ht_cauchy(x);

    while (abs(h) >= EPSILON)
    {
        h = cdf_ht_cauchy(x)/deriv_cdf_ht_cauchy(x);
        // x(i+1) = x(i) - f(x) / f'(x)
        x = x - h;
    }
    return x+value;
}

//our gamma cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_tdoa::cdf_ht_gamma(double x)
{
    // generate our random variable
    float tmp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

    //our gamma cdf
    float gamma = 0.5*(1+erf( (x-param->htg_mu())/(sqrt(2)*param->gauss_sigma())))/(1+param->htg_scale());
    if (x>0){
        //we use the regularised lower gamma function of the boost library for ease of programming
        gamma += boost::math::gamma_p(param->htg_k(), param->htg_lambda()*x)* param->htg_scale()/(1+param->htg_scale());
    }
    return gamma - tmp;
}

//the derivative of our gamma cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_tdoa::deriv_cdf_ht_gamma(double x)
{
    //our gamma cdf
    float gamma = (((1/(sqrt(2*M_PI)*param->gauss_sigma()))*exp((-(x-param->htg_mu())*(x-param->htg_mu()))
            /(2*param->gauss_sigma()*param->gauss_sigma())))/((1+param->htg_scale())*param->gauss_sigma()));
    if (x>0){
        //we use the regularised lower gamma function of the boost library for ease of programming
        gamma += boost::math::gamma_p_derivative(param->htg_k(), param->htg_lambda()*x)* param->htg_scale()/(1+param->htg_scale());
    }
    return gamma;
}

//our ht_gamma noise function, that solves for heavy tailed noise using Newton Rapson's method
float beacon_tdoa::add_ht_gamma_noise(float value){

    //our starting estimate to solve the cdf using Newton Raphson's method
    double x = 0; 
    double h = cdf_ht_gamma(x) / deriv_cdf_ht_gamma(x);
    while (abs(h) >= EPSILON)
    {
        h = cdf_ht_gamma(x)/deriv_cdf_ht_gamma(x);
        // x(i+1) = x(i) - f(x) / f'(x)
        x = x - h;
    }
        //sometimes when initialising the function a nan is thrown
    if (isnan(x + value)){
        return value;
    }else{
    return x + value;}
}


