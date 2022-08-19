#ifndef BEACON_H
#define BEACON_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>
#include "omniscient_observer.h"

/**
 * Parent class defining a beacon. The dynamic implementation is handled in children classes.
 */

class Beacon
        {
public:
    /**
     * Constructor
     */
    
    Beacon();
    /*
    std::vector<float> range_beacon(const uint16_t ID);
    std::vector<float> ground_truth(const uint16_t ID);
    void dynamic_beacon_init(const uint16_t ID);
    void dynamic_beacon_update(const uint16_t ID);
    */
    /**
     * Destructor
     */
    virtual ~Beacon() {};
    /**
   * @brief Return noisy value following gaussian distribution
   *
   * @param value value to which noise is to be added
   * @param mean mean of the gaussian noise
   * @param stddev standard deviation of the gaussian noise
   */
    virtual float add_gaussian_noise(float value, float sigma)=0;

    /**
    * @brief Return noisy value following cauchy distribution
    *
    * @param value value to which noise is to be added
    * @param mean mean of the gaussian noise
    * @param stddev standard deviation of the gaussian noise
    */
    virtual float add_ht_cauchy_noise(float value)=0;
    virtual float add_ht_gamma_noise(float value)=0;

    /**
    * @brief Output exact distances from agent to all active beacons to the terminal
    *
    * @param ID agent
    */
   // virtual void ranges_terminal(const uint16_t ID)=0;

    /**
    * @brief Output twr range from agent to all active beacons to the terminal
    *
    * @param ID agent
    */
   // virtual void measurement()=0;
    virtual void measurement(const uint16_t ID)=0;
    /*
    virtual float returnUWBdata(const uint16_t ID, float beacon)=0;
    */
    float new_logf(float a);
    float erfinvf (float a);
};


#endif //BEACON_H
