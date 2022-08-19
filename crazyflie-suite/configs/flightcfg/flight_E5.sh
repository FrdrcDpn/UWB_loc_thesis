#!/usr/bin/env bash

# Check whether pwd is project root
if [[ "${PWD##*/}" != crazyflie-suite ]]; then
    echo "Should be run from project root, exiting"
    exit 1
fi

# Run
python flight/log_flight_E5.py \
    --fileroot data \
    --filename logfin\
    --logconfig configs/logcfg/example_logcfg.json \
    --space flight/space_cyberzoo.yaml \
    --estimator kalman \
    --uwb twr \
    --trajectory tinysquare tinysquare \
    --optitrack logging \
    --optitrack_id 1 \
    --uri radio://1/50/2M/E7E7E7E7E5 \
    --ranging_algo cu  \
    --coverage lim