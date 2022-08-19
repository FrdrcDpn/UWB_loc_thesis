#include "beacon.h"
#include "agent.h"
#include "main.h"
#include <random>
#include <vector>
#include <tgmath.h>

using namespace std;
Beacon::Beacon()
{
   
}

/*
void Beacon::dynamic_beacon_init(const uint16_t ID){
  // if we use dynamic beacons, add the agent's estimated position to the dynamic beacon vector
  // if the vector is not initialised, do so

if(param->dynamic_beacons() == 1){
    mtx_bcn.lock();
  dynamic_uwb_beacon.push_back(std::vector<float>());
  dynamic_uwb_beacon[ID].push_back(0.0);
  dynamic_uwb_beacon[ID].push_back(0.0);
  dynamic_uwb_beacon[ID].push_back(1.0);
  mtx_bcn.unlock();
}

}

void Beacon::dynamic_beacon_update(const uint16_t ID){
  // if we use dynamic beacons, add the agent's estimated position to the dynamic beacon vector
  // update with estimated state
  
if(param->dynamic_beacons() == 1){
    mtx_bcn.lock();
  dynamic_uwb_beacon[ID][0] = s[ID]->state_estimate[0];
  dynamic_uwb_beacon[ID][1] = s[ID]->state_estimate[1];
  mtx_bcn.unlock();
}

}

// input agent ID, output vector of ranges to beacon
std::vector<float> Beacon::range_beacon(const uint16_t ID){
    float b1, r1;
    std::vector<float> beacon_ranges;
    //draw the range from the beacons
    OmniscientObserver obs;
    for (uint16_t i = 1; i <= 8; i++) {
        obs.uwb_beacon(ID,r1,b1,i);
        beacon_ranges.push_back(r1);
    }
    return beacon_ranges;
}

// input agent ID, output vector of ground truth of agent
std::vector<float> Beacon::ground_truth(const uint16_t ID) {
    float x, y;
    std::vector<float> ground_truth;
    x = s[ID]->get_position(0);
    y = s[ID]->get_position(1);
    ground_truth.push_back(x);
    ground_truth.push_back(y);
    return ground_truth;
}
*/
/* compute natural logarithm with a maximum error of 0.85089 ulp */
//https://stackoverflow.com/questions/27229371/inverse-error-function-in-c
float Beacon::new_logf(float a)
{
    float i = 0; 
    float m = 0;
    float r = 0; 
    float s = 0; 
    float t = 0;
    int e = 0;

    m = frexpf (a, &e);
    if (m < 0.666666667f) { // 0x1.555556p-1
        m = m + m;
        e = e - 1;
    }
    i = (float)e;
    /* m in [2/3, 4/3] */
    m = m - 1.0f;
    s = m * m;
    /* Compute log1p(m) for m in [-1/3, 1/3] */
    r =             -0.130310059f;  // -0x1.0ae000p-3
    t =              0.140869141f;  //  0x1.208000p-3
    r = fmaf (r, s, -0.121484190f); // -0x1.f19968p-4
    t = fmaf (t, s,  0.139814854f); //  0x1.1e5740p-3
    r = fmaf (r, s, -0.166846052f); // -0x1.55b362p-3
    t = fmaf (t, s,  0.200120345f); //  0x1.99d8b2p-3
    r = fmaf (r, s, -0.249996200f); // -0x1.fffe02p-3
    r = fmaf (t, m, r);
    r = fmaf (r, m,  0.333331972f); //  0x1.5554fap-2
    r = fmaf (r, m, -0.500000000f); // -0x1.000000p-1
    r = fmaf (r, s, m);
    r = fmaf (i,  0.693147182f, r); //  0x1.62e430p-1 // log(2)
    if (!((a > 0.0f) && (a <= 3.40282346e+38f))) { // 0x1.fffffep+127
        r = a + a;  // silence NaNs if necessary
        if (a  < 0.0f) r = ( 0.0f / 0.0f); //  NaN
        if (a == 0.0f) r = (-1.0f / 0.0f); // -Inf
    }
    return r;
}

/* compute inverse error functions with maximum error of 2.35793 ulp */
//https://stackoverflow.com/questions/27229371/inverse-error-function-in-c
float Beacon::erfinvf (float a)
{
    float p = 0; 
    float r = 0; 
    float t = 0;
    t = fmaf (a, 0.0f - a, 1.0f);
    t = new_logf(t);
    if (fabsf(t) > 6.125f) { // maximum ulp error = 2.35793
        p =              3.03697567e-10f; //  0x1.4deb44p-32
        p = fmaf (p, t,  2.93243101e-8f); //  0x1.f7c9aep-26
        p = fmaf (p, t,  1.22150334e-6f); //  0x1.47e512p-20
        p = fmaf (p, t,  2.84108955e-5f); //  0x1.dca7dep-16
        p = fmaf (p, t,  3.93552968e-4f); //  0x1.9cab92p-12
        p = fmaf (p, t,  3.02698812e-3f); //  0x1.8cc0dep-9
        p = fmaf (p, t,  4.83185798e-3f); //  0x1.3ca920p-8
        p = fmaf (p, t, -2.64646143e-1f); // -0x1.0eff66p-2
        p = fmaf (p, t,  8.40016484e-1f); //  0x1.ae16a4p-1
    } else { // maximum ulp error = 2.35002
        p =              5.43877832e-9f;  //  0x1.75c000p-28
        p = fmaf (p, t,  1.43285448e-7f); //  0x1.33b402p-23
        p = fmaf (p, t,  1.22774793e-6f); //  0x1.499232p-20
        p = fmaf (p, t,  1.12963626e-7f); //  0x1.e52cd2p-24
        p = fmaf (p, t, -5.61530760e-5f); // -0x1.d70bd0p-15
        p = fmaf (p, t, -1.47697632e-4f); // -0x1.35be90p-13
        p = fmaf (p, t,  2.31468678e-3f); //  0x1.2f6400p-9
        p = fmaf (p, t,  1.15392581e-2f); //  0x1.7a1e50p-7
        p = fmaf (p, t, -2.32015476e-1f); // -0x1.db2aeep-3
        p = fmaf (p, t,  8.86226892e-1f); //  0x1.c5bf88p-1
    }
    r = a * p;
    return r;
}

