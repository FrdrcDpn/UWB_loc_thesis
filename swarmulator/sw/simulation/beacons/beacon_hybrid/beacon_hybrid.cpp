#include "beacon_hybrid.h"
#include "agent.h"
#include "main.h"
#include <stdio.h>
#include <boost/math/special_functions/gamma.hpp>
using namespace std;
#define EPSILON 0.001

beacon_hybrid::beacon_hybrid() {
beacon_alg = "beacon_hybrid";
next_UWB_measurement_time = simtime_seconds;
est = param->htg_mu();
tdoa_noise = param->gauss_sigma_twr(); 
twr_noise = param->gauss_sigma_twr(); 

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
void beacon_hybrid::measurement(const uint16_t ID){
    
    float x_0,y_0,x_1,y_1,dx0,dy0,dx1,dy1,d0,d1,dd, d,x_a_0,y_a_0,x_a_1,y_a_1,cov1, cov2, cov3, cov4;

    // initial values
    bool static_beacon_1 = false;
    bool dynamic_beacon_1 = false;
    bool static_beacon_2 = false;
    bool dynamic_beacon_2 = false;
    bool beacon_1_selected = false; 
    bool beacon_2_selected = false; 
    
    bool static_tdoa = false; 
    bool dynamic_twr = false; 

    mtx_e.lock();
    // we make a copy of beacon state vector at moment of measurement (freeze in time)
    std::vector<Beacon_gen *> b_0 = b; 
    mtx_e.unlock();
    // Now we random shuffle the vector to make sure a random broadcasting beacon is selected
    std::random_shuffle (b_0.begin(), b_0.end() ); 

    // Now we loop over the shuffeled beacon state vector and we select 2 enabled and broadcasting beacons
    for (uint16_t ID_b = 0; ID_b < b_0.size(); ID_b++) {
    // first select beacon 1 
        
        // is it broadcasting, we have not selected a beacon yet, it is not our own ID
        if(b_0[ID_b]->state_b[4] == 1.0 && beacon_1_selected == false && ID+8!=b_0[ID_b]->state_b[6]){

           //if we have a static beacon
           if(b_0[ID_b]->state_b[5]==0.0){
            static_tdoa = true; 
            x_0 = b_0[ID_b]->state_b[0]; // x-location of static beacon from its state vector
            y_0 = b_0[ID_b]->state_b[1]; // y-location of static beacon from its state vector
            x_a_0 = x_0; // for the static beacons we use the static x location as anchor x coordinate
            y_a_0 = y_0; // for the static beacons we use the static y location as anchor y coordinate
           }else if (b_0[ID_b]->state_b[5]==1.0){
            dynamic_twr = true; 
            cov1 = b_0[ID_b]->state_b[9]; // get the covariance value of the dynamic beacon
            cov2 = b_0[ID_b]->state_b[10]; // get the covariance value of the dynamic beacon
            cov3 = b_0[ID_b]->state_b[11]; // get the covariance value of the dynamic beacon
            cov4 = b_0[ID_b]->state_b[12]; // get the covariance value of the dynamic beacon
            x_0 = b_0[ID_b]->state_b[7]; // x-location of dynamic beacon from desired trajectory
            y_0 = b_0[ID_b]->state_b[8]; // y-location of dynamic beacon from desired trajectory
            x_a_0 = b_0[ID_b]->state_b[0]; // for the dynamic beacons we use the dymamic x state estimate as anchor x coordinate
            y_a_0 = b_0[ID_b]->state_b[1]; // for the dynamic beacons we use the dymamic y state estimate as anchor y coordinate

            beacon_1_selected = true; // we have selected the beacon
                static_beacon_1 = !bool(b_0[ID_b]->state_b[5]); // 5th entry of state vector is 0.0 if it is a static beacon, 1.0 if it is dynamic
                dynamic_beacon_1 = bool(b_0[ID_b]->state_b[5]);
                sel_beacon_1 = int(b_0[ID_b]->state_b[6]); //6h entry of state vector is the beacon ID 
           }
           // generate range measurements
           dx0 = s[ID]->state[0] - x_0;
           dy0 = s[ID]->state[1] - y_0;
           d0 = sqrt(dx0*dx0 + dy0*dy0);

           // only continue with selected beacons if in range
           if(d0<=param->max_UWB_range() && static_tdoa == true){
                beacon_1_selected = true; // we have selected the beacon
                static_beacon_1 = !bool(b_0[ID_b]->state_b[5]); // 5th entry of state vector is 0.0 if it is a static beacon, 1.0 if it is dynamic
                dynamic_beacon_1 = bool(b_0[ID_b]->state_b[5]);
                sel_beacon_1 = int(b_0[ID_b]->state_b[6]); //6h entry of state vector is the beacon ID 
                 
           }else if(d0>param->max_UWB_range()&& static_tdoa == true){
               //else we discard our selected beacons
               dynamic_twr = false; 
               static_tdoa = false; 
               beacon_1_selected = false; 
           }

        }

        // now select beacon 2 if we do not perform twr with a dynamic beacon
         
        //if static tdoa is true, if beacon 2 is not same as 1, if it is static beacon, and broadcasting, and we have not selected a 2nd beacon yet
        if(static_tdoa == true && sel_beacon_1 != int(b_0[ID_b]->state_b[6]) && b_0[ID_b]->state_b[5]==0.0 && b_0[ID_b]->state_b[4] == 1.0 && beacon_2_selected == false){
            x_1 = b_0[ID_b]->state_b[0]; // x-location of static beacon from its state vector
            y_1 = b_0[ID_b]->state_b[1]; // y-location of static beacon from its state vector
            x_a_1 = x_1; // for the static beacons we use the static x location as anchor x coordinate
            y_a_1 = y_1; // for the static beacons we use the static y location as anchor y coordinate
            // generate tdoa measurements
            dx1 = s[ID]->state[0] - x_1;
            dy1 = s[ID]->state[1] - y_1;
            d1 = sqrt(dx1*dx1 + dy1*dy1);

            //only continue if the beacon is in range
            if(d1<=param->max_UWB_range()){
                beacon_2_selected = true; // we have selected the beacon
                static_beacon_2 = !bool(b_0[ID_b]->state_b[5]); // 5th entry of state vector is 0.0 if it is a static beacon, 1.0 if it is dynamic
                dynamic_beacon_2 = bool(b_0[ID_b]->state_b[5]); 
                sel_beacon_2 = int(b_0[ID_b]->state_b[6]); //6h entry of state vector is the beacon ID 
                
            }
           
        }

        if(beacon_1_selected == true && dynamic_twr == true){
            
            break;
        }
        if(beacon_2_selected == true && static_tdoa == true){
            
            break;
        }
    }
    // Now we loop over the shuffeled beacon state vector and we select 2 enabled and broadcasting beacons
  //  for (uint16_t ID_b = 0; ID_b < b_0.size(); ID_b++) {
        
      //  }
   
    
    // if we have selected 2 available beacons during this measurement cycle, continue with tdoa 
    if(static_tdoa == true && dynamic_twr == false && beacon_1_selected == true && beacon_2_selected == true && simtime_seconds>=next_UWB_measurement_time){
        next_UWB_measurement_time = next_UWB_measurement_time + 1.0/param->UWB_frequency();
if(param->terminaloutput()==1.0){
                std::cout<<"agent "<<ID<<" ranges with beacon "<<sel_beacon_1<<std::endl; // output the selected beacons to terminal (can be commented)
                std::cout<<"agent "<<ID<<" ranges with beacon "<<sel_beacon_2<<std::endl; // output the selected beacons to terminal (can be commented)
                }
    // generate tdoa measurements
    dd = abs(d1)-abs(d0);

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
        float noisy_meas = add_gaussian_noise(dd, param->gauss_sigma_tdoa());
        s[ID]->UWBm.at(0) = noisy_meas; 

        v[i] = noisy_meas-dd; 
        i = i +1; 
        if (i > 9){
            i = 0;
        }
        
        double sum = std::accumulate(v.begin(), v.end(), 0.0);
        double mean = sum / v.size();

        double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / v.size() - mean * mean);
        
        if(simtime_seconds > 0.5){
        tdoa_noise = float(stdev); 
        }
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
    s[ID]->UWBm[5] = 1 ;
    s[ID]->UWBm[6] = 1 ; // we have pushed a tdoa measurement let ekf know
   // beacon_measurement[ID].push_back({1});
   // mtx_e.unlock();
    
   }

   // if we have selected 1 available dynamic beacon during this measurement cycle, continue twr
    if(dynamic_twr == true && static_tdoa == false && beacon_1_selected == true && simtime_seconds>=next_UWB_measurement_time){
    next_UWB_measurement_time = next_UWB_measurement_time + 1.0/param->UWB_frequency();
    //initialise uwb dataset to be used by EKF
   // mtx_bcn.lock();
   // UWB.push_back(std::vector<std::vector<float>>());
   // UWB[ID].push_back(std::vector<float>());
if(param->terminaloutput()==1.0){
                std::cout<<"agent "<<ID<<" dynamic ranges with agent "<<sel_beacon_1-8<<std::endl; // output the selected beacons to terminal (can be commented)
                }
    //add preferred noise on top of tdoa measurements, defined in parameters file
    if (param->noise_type() == 0){
        // twr measurement, x beacon, y beacon, simulation time
        s[ID]->UWBm[0] = d0;
        s[ID]->UWBm[1] = x_a_0;
        s[ID]->UWBm[2] = y_a_0;
       
       // UWB[ID].push_back({d,x_a_0, y_a_0, simtime_seconds});
       // mtx_bcn.unlock();
    }
    else if (param->noise_type() == 1){
        float noisy_meas =add_gaussian_noise(d0, param->gauss_sigma_twr());
        s[ID]->UWBm[0] = noisy_meas; 
        s[ID]->UWBm[1] = x_a_0;
        s[ID]->UWBm[2] = y_a_0;

        v1[i] = noisy_meas-d0; 
        i1 = i1 +1; 
        if (i1 > 9){
            i1 = 0;
        }
        
        double sum = std::accumulate(v1.begin(), v1.end(), 0.0);
        double mean = sum / v1.size();

        double sq_sum = std::inner_product(v1.begin(), v1.end(), v1.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / v1.size() - mean * mean);

        if(simtime_seconds > 0.5){
        twr_noise = float(stdev); 
        }
        
        // twr measurement, x beacon, y beacon, simulation time
       // UWB[ID].push_back({add_gaussian_noise(d),x_a_0, y_a_0, simtime_seconds});
       // mtx_bcn.unlock();
    }
    else if (param->noise_type() == 2){
        s[ID]->UWBm[0] = add_ht_cauchy_noise(d0);
        s[ID]->UWBm[1] = x_a_0;
        s[ID]->UWBm[2] = y_a_0;
        // twr measurement, x beacon, y beacon, simulation time
       // UWB[ID].push_back({add_ht_cauchy_noise(d),x_a_0, y_a_0, simtime_seconds});
      //  mtx_bcn.unlock();
    }
    else if (param->noise_type() == 3){
        s[ID]->UWBm[0] = add_ht_gamma_noise(d0);
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


    //also pass the covariance matrix x and y values of the dynamic beacon
    s[ID]->UWBm[7] = cov1;
    s[ID]->UWBm[8] = cov2;
    s[ID]->UWBm[9] = cov3;
    s[ID]->UWBm[10] = cov4;
   s[ID]->UWBm[5] = 1 ;
   s[ID]->UWBm[6] = 0 ; // we have pushed a twr measurement let ekf know
   }
}
   
// function to add gaussian noise to UWB measurements
float beacon_hybrid::add_gaussian_noise(float value, float sigma) {
    float noisy_value;
    float mean = param->htg_mu();
  
    // Random seed
    random_device rd;

    // Initialize Mersenne Twister pseudo-random number generator
    mt19937 gen(rd());
    std::normal_distribution<float> dis(mean, sigma);
    // Add Gaussian noise
    noisy_value = value + dis(gen);

    return noisy_value;
}

//our cauchy cdf function to be used with our Newton Raphson solving method, to solve for noise
float beacon_hybrid::cdf_ht_cauchy(float x)
{
   // generate our random variable
   // float tmp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    //gaussian cdf
    float gauss = 0.5*(1+boost::math::erf( x/(sqrt(2)*param->gauss_sigma()))) * (1-param->htc_ratio());
    //cauchy cdf
    float cauchy = (((1/M_PI) * atan(x/param->htc_gamma())) + 0.5) * param->htc_ratio();
    //return ht_cauchy cdf
    return gauss + cauchy;
}

// the derivative of our cauchy cdf function to be used with our Newton Raphson solving method, to solve for noise
float beacon_hybrid::deriv_cdf_ht_cauchy(float x)
{

    //float gauss = (exp(-pow((x-param->htg_mu()),2)/(2*pow(param->gauss_sigma(),2)) )/ (param->gauss_sigma() * sqrt(2*M_PI)))/((1+param->htg_scale())*param->gauss_sigma());

    float gauss = (1-param->htc_ratio())*(1/((sqrt(2*M_PI))*param->gauss_sigma()))*exp((-(x*x)/(2*param->gauss_sigma()*param->gauss_sigma())));
    //cauchy cdf
    float cauchy = (param->htc_gamma()*param->htc_ratio())/((M_PI*x*x)+(param->htc_gamma()*param->htc_gamma()*M_PI));
    //return ht_cauchy cdf
    return gauss + cauchy;
}

//our ht_cauchy noise function, that solves for heavy tailed noise using Newton Rapson's method
float beacon_hybrid::add_ht_cauchy_noise(float value){

    //our starting estimate to solve the cdf using Newton Raphson's method
     std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(0.0, 1.0);
    //our starting estimate to solve the cdf using Newton Raphson's method
    float noise = dis(gen);
    float x = est;
    
    float delta = 100; 
    while (delta >= 0.001)
    {
        float h = (cdf_ht_cauchy(x)-noise) / deriv_cdf_ht_cauchy(x);
        
        // x(i+1) = x(i) - f(x) / f'(x)
        float newx = x - h;
        //std::cout<<"h "<<h<<std::endl;
        delta = abs(x-newx);
        x = newx; 
    }

    //sometimes when no solution can be found a nan is thrown 
    if (isnan(x + value)){
        return value;
    }else{
    return x + value;
}}

//our gamma cdf function to be used with our Newton Raphson solving method, to solve for noise
float beacon_hybrid::cdf_ht_gamma(float x)
{
    // generate our random variable
   // float tmp = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

  //  float tmp = 0.5 * boost::math::erf(-(x-param->htg_mu())/(param->gauss_sigma()*sqrt(2)))/(1+param->htg_scale());

    // verified
   float tmp = 0.5*(1+boost::math::erf((x-param->htg_mu())/(sqrt(2)*param->gauss_sigma())))/(1+param->htg_scale());  
    
    if (x>0){
        //we use the regularised lower gamma function of the boost library for ease of programming
       tmp += boost::math::gamma_p(param->htg_k(), param->htg_lambda()*x)* param->htg_scale()/(1+param->htg_scale());
    }
    return tmp;
}

//the derivative of our gamma cdf function to be used with our Newton Raphson solving method, to solve for noise
float beacon_hybrid::deriv_cdf_ht_gamma(float x)
{
    //our gamma cdf

     
  float tmp = (exp(-pow((x-param->htg_mu()),2)/(2*pow(param->gauss_sigma(),2)) )/ (param->gauss_sigma() * sqrt(2*M_PI)))*((1+param->htg_scale()));//*param->gauss_sigma());

   // float tmp = (((1/(sqrt(2*M_PI)*param->gauss_sigma()))*exp((-(x-param->htg_mu())*(x-param->htg_mu()))/(2*param->gauss_sigma()*param->gauss_sigma())))/((1+param->htg_scale())*param->gauss_sigma()));
    
  //  float tmp = ( 1 / ( param->gauss_sigma() * sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (x-param->htg_mu())/param->gauss_sigma(), 2.0 ) )  /(1+param->htg_scale());
//our gamma cdf
  //  float tmp = (((1/(sqrt(2*M_PI)*param->gauss_sigma()))*exp((-(x-param->htg_mu())*(x-param->htg_mu()))/(2*param->gauss_sigma()*param->gauss_sigma())))/((1+param->htg_scale())*param->gauss_sigma()));
    if (x>0){
        //we use the regularised lower gamma function of the boost library for ease of programming
       tmp += boost::math::gamma_p_derivative(param->htg_k(), param->htg_lambda()*x)*param->htg_lambda()* param->htg_scale()/(1+param->htg_scale());
    }
    return tmp ;
}

//our ht_gamma noise function, that solves for heavy tailed noise using Newton Rapson's method
float beacon_hybrid::add_ht_gamma_noise(float value){

    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<float> dis(0, 1.0);
    //our starting estimate to solve the cdf using Newton Raphson's method
    //float x = dis(gen);
    //float h = cdf_ht_gamma(x) / deriv_cdf_ht_gamma(x);
    //while (abs(h) >= EPSILON)
    //{
    //    h = cdf_ht_gamma(x)/deriv_cdf_ht_gamma(x);
        // x(i+1) = x(i) - f(x) / f'(x)
    //    x = x - h;
    //}
    float noise = dis(gen);//static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float x = est;
   // float h = cdf_ht_gamma(x) / deriv_cdf_ht_gamma(x);

    float delta = 100; 
    while (delta >= 0.001)
    {
        float h = (cdf_ht_gamma(x)-noise)/ (deriv_cdf_ht_gamma(x));

        
        // x(i+1) = x(i) - f(x) / f'(x)
        float newx =  x - h;
        //std::cout<<"h "<<h<<std::endl;
        delta = abs(x-newx);
        x = newx; 
    }
    
    //sometimes when no solution can be found a nan is thrown 
    if (isnan(x + value)){
        return value;
    }else{
    return x + value;
}}


