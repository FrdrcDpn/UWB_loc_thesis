#ifndef BEACON_GEN_H
#define BEACON_GEN_H

#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdint.h>

/**
 * Parent class defining a beacon. The dynamic implementation is handled in children classes.
 */

class Beacon_gen
        {
public:
    /**
     * Constructor
     */
    Beacon_gen(int i, std::vector<float> b);

    //function updating the broadcasting variable of state_b
    std::vector<float> beacon_status_update(std::vector<float>);

    //function updating the location of dynamic beacons
    std::vector<float> beacon_dynamic_state_update(std::vector<float>);

    std::vector<float> state_b; // State vector beacon
    uint16_t ID_b; // ID of beacon
    float next_measurement_time = 0;
    float end_measurement_time = 0;
    
    /**
     * Destructor
     */
    virtual ~Beacon_gen() {};
    
};

#endif //BEACON_GEN_H
