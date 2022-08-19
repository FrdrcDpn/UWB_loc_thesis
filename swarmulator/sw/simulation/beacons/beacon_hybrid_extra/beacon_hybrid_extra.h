#ifndef BEACON_HYBRID_EXTRA_H
#define BEACON_HYBRID_EXTRA_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "beacon.h"

class beacon_hybrid_extra: public Beacon
{

public:
    /**
     * Construction. beacon_hybrid_extra is a child class of Beacon.
     */
    beacon_hybrid_extra();
    float next_UWB_measurement_time = 0;
    float next_UWB_D_measurement_time = 0;
    float sel_beacon_1 ; 
    float sel_beacon_2 ; 
  
/*
    // function to output some range info to terminal (not useful to be removed)
    void ranges_terminal(const uint16_t ID);
    
    // function to return some UWB data (not useful to be removed)
    float returnUWBdata(const uint16_t ID, float beacon);
*/
    // our HYBRID measurement function
    void measurement(const uint16_t ID);

    float est; 
    // function to add gaussian noise to measurements
    float add_gaussian_noise(float value, float sigma);

    // function to ht cauchy noise to measurements
    float add_ht_cauchy_noise(float value);
    float cdf_ht_cauchy(float x);
    float deriv_cdf_ht_cauchy(float x);
    std::vector<float> v{0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0};
    std::vector<float> v1{0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0,0, 0, 0, 0, 0};
    int i = 0;
    int i1 = 0;
    // function to add gamma noise to measurements
    float add_ht_gamma_noise(float value);
    float cdf_ht_gamma(float x);
    float deriv_cdf_ht_gamma(float x);
};


#endif //BEACON_HYBRID_EXTRA_H
