#ifndef BEACON_TWR_H
#define BEACON_TWR_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include "beacon.h"

class beacon_twr: public Beacon
{


public:
    float next_UWB_measurement_time;
    /**
     * Construction. beacon_twr is a child class of Beacon.
     */
    beacon_twr();
/*
    // function to output some range info to terminal (not useful to be removed)
    void ranges_terminal(const uint16_t ID);

    // function to return some UWB data (not useful to be removed)
    float returnUWBdata(const uint16_t ID, float beacon);
    */
    // our TWR measurement function
    void measurement(const uint16_t ID);

    // function to add gaussian noise to measurements
    float add_gaussian_noise(float value, float sigma);

    // function to ht cauchy noise to measurements
    float add_ht_cauchy_noise(float value);
    float add_ht_gamma_noise(float value);

    // function to add gamma noise to measurements
    double cdf_ht_gamma(double x);
    double deriv_cdf_ht_gamma(double x);
};


#endif //BEACON_TWR_H
