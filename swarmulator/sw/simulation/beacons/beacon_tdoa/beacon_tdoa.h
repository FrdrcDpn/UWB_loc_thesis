#ifndef BEACON_TDOA_H
#define BEACON_TDOA_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "beacon.h"

class beacon_tdoa: public Beacon
{

public:
    /**
     * Construction. beacon_tdoa is a child class of Beacon.
     */
    beacon_tdoa();
    float next_UWB_measurement_time = 0;
    float sel_beacon_1 = 0; 
  
/*
    // function to output some range info to terminal (not useful to be removed)
    void ranges_terminal(const uint16_t ID);
    
    // function to return some UWB data (not useful to be removed)
    float returnUWBdata(const uint16_t ID, float beacon);
*/
    // our TDOA measurement function
    void measurement(const uint16_t ID);

    // function to add gaussian noise to measurements
    float add_gaussian_noise(float value, float sigma);

    // function to ht cauchy noise to measurements
    float add_ht_cauchy_noise(float value);
    double cdf_ht_cauchy(double x);
    double deriv_cdf_ht_cauchy(double x);

    // function to add gamma noise to measurements
    float add_ht_gamma_noise(float value);
    double cdf_ht_gamma(double x);
    double deriv_cdf_ht_gamma(double x);
};


#endif //BEACON_TDOA_H
