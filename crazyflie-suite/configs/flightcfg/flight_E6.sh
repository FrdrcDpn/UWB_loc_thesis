#!/usr/bin/env bash

# Check whether pwd is project root
if [[ "${PWD##*/}" != crazyflie-suite ]]; then
    echo "Should be run from project root, exiting"
    exit 1
fi

# Run
python flight/log_flight_E6.py \
    --fileroot data \
    --filename dlog\
    --logconfig configs/logcfg/example_logcfg.json \
    --space flight/space_cyberzoo.yaml \
    --estimator kalman \
    --uwb twr \
    --trajectory square square \
    --optitrack none \
    --optitrack_id 1 \
    --uri radio://0/120/2M/E7E7E7E7E6 \
    --ranging_algo cu \
    --coverage full 
