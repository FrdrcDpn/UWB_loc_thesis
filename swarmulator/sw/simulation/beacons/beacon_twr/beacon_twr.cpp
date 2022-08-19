#include "beacon_twr.h"
#include "agent.h"
#include "main.h"
#include "txtwrite.h"
#include <boost/math/special_functions/gamma.hpp>
#include <random>
#define EPSILON 0.001
using namespace std;

/*
*in parameters file, change params file to define the noise type
*noise_type 0 = no noise
*noise_type 1 = gaussian noise
*noise_type 2 = heavy tailed cauchy noise
*noise_type 3 = heavy tailed gamma noise
*/

beacon_twr::beacon_twr() {
beacon_alg = "beacon_twr";
next_UWB_measurement_time = 0;
}
/*
// function to output some range info to terminal (not useful to be removed)
void beacon_twr::ranges_terminal(const uint16_t ID){
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
float beacon_twr::returnUWBdata(const uint16_t ID, float beacon){
    // function that returns the latest distance entry of uwb data
    mtx_bcn.lock();
   float dist = UWB[ID].back()[0];
    mtx_bcn.unlock();
    return dist;
}
*/
// measurement function, called by controller at simulation frequency
// constructs UWB measurement from available UWB beacons
void beacon_twr::measurement(const uint16_t ID){
    float x_0, y_0, dx0, dy0, d, x_a_0, y_a_0;

    // initial values
    bool static_beacon_1 = false;
    bool dynamic_beacon_1 = false;
    bool beacon_1_selected = false; 


    // we make a copy of beacon state vector at moment of measurement (freeze in time)
    std::vector<Beacon_gen *> b_0 = b; 

    // Now we random shuffle the vector to make sure a random broadcasting beacon is selected
    std::random_shuffle (b_0.begin(), b_0.end() ); 

    // Now we loop over the shuffeled beacon state vector and we select 1 enabled and broadcasting beacon
    for (uint16_t ID_b = 0; ID_b < b_0.size(); ID_b++) {
     
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
           float sel_beacon_1 = int(b_0[ID_b]->state_b[6]); //6h entry of state vector is the beacon ID 
           if(param->terminaloutput()==1.0){
           std::cout<<"agent "<<ID<<" ranges with beacon "<<sel_beacon_1<<std::endl; // output the selected beacons to terminal (can be commented)
           }
        }
   }
   
    // if we have selected 1 available beacon during this measurement cycle, continue
    if(beacon_1_selected == true&& simtime_seconds>=next_UWB_measurement_time){
     next_UWB_measurement_time = next_UWB_measurement_time + 1.0/param->UWB_frequency();
    //generate twr measurements
    dx0 = s[ID]->state[0] - x_0;
    dy0 = s[ID]->state[1] - y_0;
    d = sqrt(dx0*dx0 + dy0*dy0);

    //initialise uwb dataset to be used by EKF
   // mtx_bcn.lock();
   // UWB.push_back(std::vector<std::vector<float>>());
   // UWB[ID].push_back(std::vector<float>());

    //add preferred noise on top of tdoa measurements, defined in parameters file
    if (param->noise_type() == 0){
        // twr measurement, x beacon, y beacon, simulation time
        s[ID]->UWBm[0] = d;
        s[ID]->UWBm[1] = x_a_0;
        s[ID]->UWBm[2] = y_a_0;
       // UWB[ID].push_back({d,x_a_0, y_a_0, simtime_seconds});
       // mtx_bcn.unlock();
    }
    else if (param->noise_type() == 1){
        s[ID]->UWBm[0] = add_gaussian_noise(d, param->gauss_sigma_twr());
        s[ID]->UWBm[1] = x_a_0;
        s[ID]->UWBm[2] = y_a_0;
        // twr measurement, x beacon, y beacon, simulation time
       // UWB[ID].push_back({add_gaussian_noise(d),x_a_0, y_a_0, simtime_seconds});
       // mtx_bcn.unlock();
    }
    else if (param->noise_type() == 2){
        s[ID]->UWBm[0] = add_ht_cauchy_noise(d);
        s[ID]->UWBm[1] = x_a_0;
        s[ID]->UWBm[2] = y_a_0;
        // twr measurement, x beacon, y beacon, simulation time
       // UWB[ID].push_back({add_ht_cauchy_noise(d),x_a_0, y_a_0, simtime_seconds});
      //  mtx_bcn.unlock();
    }
    else if (param->noise_type() == 3){
        s[ID]->UWBm[0] = add_ht_gamma_noise(d);
        s[ID]->UWBm[1] = x_a_0;
        s[ID]->UWBm[2] = y_a_0;
        // twr measurement, x beacon, y beacon, simulation time
       // UWB[ID].push_back({add_ht_gamma_noise(d),x_a_0, y_a_0, simtime_seconds});
       // mtx_bcn.unlock();
    }
    if(param->terminaloutput()==1.0){
    // output to terminal whether the selected beacons are dynamic or static (can be commented)
    std::cout<<"dynamic beacon_1 "<<dynamic_beacon_1<<" static beacon_1 "<<static_beacon_1<<std::endl;
    }
    // if during this cycle a measurement is available, let the EKF know by pusing a 1 
  //  beacon_measurement.push_back(std::vector<std::vector<float>>());
   // beacon_measurement[ID].push_back(std::vector<float>());
   // beacon_measurement[ID].push_back({1});

   s[ID]->UWBm[5] = 1 ;
    
   }
}

// function to add gaussian noise to UWB measurements
float beacon_twr::add_gaussian_noise(float value, float sigma) {
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

//our ht_cauchy noise function, that solves for heavy tailed noise 
float beacon_twr::add_ht_cauchy_noise(float value){
    float cauchy_alpha = (2*M_PI*param->htc_gamma()) / (sqrt(2*M_PI*pow(param->gauss_sigma(),2)) + M_PI*param->htc_gamma());
    float CDF_limit = (2-cauchy_alpha)*0.5;
    float tmp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    if (tmp < CDF_limit){
        //Gaussian CDF: F(x) = (1/2) * (1 + erf( (x-mu)/sqrt(2*sig^2) ))
        return value + sqrt(2*pow(param->gauss_sigma(),2))*erfinvf(2*tmp/(2-cauchy_alpha)-1);
    }
    else{
        //Cauchy CDF: F(x) = (1/pi) * arctan((x-x0)/gamma) + 1/2
        //Because of scaling with alpha (Area under CDF no longer 1), need to also start from -inf and then flip
        tmp = 1-tmp;
        return value -param->htc_gamma()*tan(M_PI*(tmp/cauchy_alpha-0.5));
    }
}

//our gamma cdf function to be used with our Newton Raphson solving method, to solve for noise
double beacon_twr::cdf_ht_gamma(double x)
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
double beacon_twr::deriv_cdf_ht_gamma(double x)
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
float beacon_twr::add_ht_gamma_noise(float value){
    double x = 0; //our starting estimate to solve the cdf using Newton Raphson's method
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


